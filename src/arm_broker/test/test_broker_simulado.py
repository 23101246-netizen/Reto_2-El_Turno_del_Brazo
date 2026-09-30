""" Pruebas del broker y del cliente con ROS 2 SIMULADO — Ítem 2 del Reto 2 """

"""No necesitan ROS 2 ni el robot: se sustituyen rclpy y las interfaces por dobles de prueba"""
"""El doble de ServerGoalHandle copia las reglas de rclpy (Humble) que importan aquí:
 - abort() y succeed() solo son válidos desde EXECUTING (o CANCELING)
 - canceled() solo es válido desde CANCELING
 - el Result solo llega al cliente si el goal pasó por execute() -> execute_callback"""
"""Cubre: goal válido, rechazos (límites, workspace, paso), cola llena, cancelación en cola y en
ejecución, nunca más de un goal EXECUTING, órdenes de FIFO y Round Robin, y que nadie más
publique en /joint_states

    cd src/arm_broker && python3 -m unittest discover -s test -v
"""
import ast
import csv
import enum
import glob
import os
import sys
import tempfile
import threading
import time
import types
import unittest
import uuid

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, RAIZ)


# 1. Dobles de rclpy y de las interfaces
class Fut:
    """Future mínimo: result(), done(), add_done_callback()"""

    def __init__(self):
        self._ev = threading.Event()
        self._val = None
        self._cbs = []

    def set_result(self, v):
        self._val = v
        self._ev.set()
        for cb in self._cbs:
            cb(self)

    def done(self):
        return self._ev.is_set()

    def result(self):
        return self._val

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self._ev.is_set():
            cb(self)


class Param:
    def __init__(self, v):
        self.value = v


class Log:
    def __init__(self):
        self.lineas = []

    def _add(self, nivel, m):
        self.lineas.append((nivel, m))

    def info(self, m, **k):
        self._add('I', m)

    def warn(self, m, **k):
        self._add('W', m)

    def error(self, m, **k):
        self._add('E', m)


class Node:
    PARAMS = {}     # Valores que reemplazan a los declarados por defecto

    def __init__(self, name):
        self._p = {}
        self._log = Log()

    def declare_parameter(self, k, v):
        self._p[k] = Param(Node.PARAMS.get(k, v))

    def get_parameter(self, k):
        return self._p[k]

    def get_logger(self):
        return self._log

    def create_publisher(self, tipo, nombre, cola):
        return Pub()

    def create_timer(self, *a, **k):
        pass

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class GoalResponse(enum.Enum):
    ACCEPT = 1
    REJECT = 2


class CancelResponse(enum.Enum):
    ACCEPT = 1
    REJECT = 2


class Obj:
    def __init__(self, **k):
        self.__dict__.update(k)


class Goal(Obj):
    def __init__(self):
        self.joint_positions = []
        self.client_id = ''
        self.priority = 0


class Result(Obj):
    def __init__(self):
        self.success = False
        self.message = ''
        self.wait_time_s = 0.0
        self.exec_time_s = 0.0


class Feedback(Obj):
    def __init__(self):
        self.state = ''
        self.queue_position = 0
        self.elapsed_s = 0.0


class MoveArm:
    Goal = Goal
    Result = Result
    Feedback = Feedback


class JointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.name = []
        self.position = []


def _modulo(nombre, **atributos):
    m = types.ModuleType(nombre)
    m.__dict__.update(atributos)
    sys.modules[nombre] = m
    return m


def instalar_dobles():
    """Se instalan los dobles solo si ROS 2 no está: con ROS real estas pruebas se omiten"""
    _modulo('rclpy',
            ok=lambda: True,
            spin_once=lambda nodo, timeout_sec=0.0: time.sleep(min(timeout_sec, 0.01)),
            spin_until_future_complete=lambda nodo, fut: fut._ev.wait(10))
    _modulo('rclpy.action', ActionServer=lambda *a, **k: None, ActionClient=lambda *a, **k: None,
            CancelResponse=CancelResponse, GoalResponse=GoalResponse)
    _modulo('rclpy.callback_groups', MutuallyExclusiveCallbackGroup=object,
            ReentrantCallbackGroup=object)
    _modulo('rclpy.executors', MultiThreadedExecutor=object)
    _modulo('rclpy.node', Node=Node)
    _modulo('sensor_msgs')
    _modulo('sensor_msgs.msg', JointState=JointState)
    _modulo('arm_broker_interfaces')
    _modulo('arm_broker_interfaces.action', MoveArm=MoveArm)
    _modulo('arm_broker_interfaces.msg', QueueState=Obj)


try:
    import rclpy  # noqa: F401
    HAY_ROS = True
except ImportError:
    instalar_dobles()
    HAY_ROS = False

if not HAY_ROS:
    from arm_broker import broker as B          # noqa: E402
    from arm_broker import cliente as C         # noqa: E402


# 2. Doble del ServerGoalHandle con la máquina de estados de rclpy
class GH:
    def __init__(self, br, goal):
        self.request = goal
        self.goal_id = types.SimpleNamespace(uuid=uuid.uuid4().bytes)
        self.state = 'accepted'
        self.br = br
        self.feedbacks = []
        self.result_future = Fut()       # Solo lo completa execute(): igual que rclpy
        self.terminado = threading.Event()

    @property
    def is_cancel_requested(self):
        return self.state == 'canceling'

    def cancelar(self):
        """Lo que hace rclpy cuando el cliente pide cancelar y cancel_callback acepta"""
        if self.state in ('accepted', 'executing'):
            self.state = 'canceling'

    def execute(self):
        # Como rclpy: con cancelación pedida no hay transición a EXECUTING, pero sí se llama
        # a execute_callback
        if self.state == 'accepted':
            self.state = 'executing'
        elif self.state != 'canceling':
            raise RuntimeError(f'execute() inválido desde {self.state}')

        def correr():
            try:
                res = self.br.execute_callback(self)
            except Exception:
                res = Result()
            if self.state in ('executing', 'canceling'):
                self.state = 'aborted'      # rclpy: «Goal state not set, assuming aborted»
            self.terminado.set()
            self.result_future.set_result(types.SimpleNamespace(result=res))
        threading.Thread(target=correr, daemon=True).start()

    def succeed(self):
        if self.state not in ('executing', 'canceling'):
            raise RuntimeError(f'succeed() inválido desde {self.state}')
        self.state = 'succeeded'

    def abort(self):
        if self.state not in ('executing', 'canceling'):
            raise RuntimeError(f'abort() inválido desde {self.state}')
        self.state = 'aborted'

    def canceled(self):
        if self.state != 'canceling':
            raise RuntimeError(f'canceled() inválido desde {self.state}')
        self.state = 'canceled'

    def publish_feedback(self, f):
        self.feedbacks.append(f.state)


# 3. Utilidades
OK = [0.3, -0.4, 0.4, 0.0, 0.3, 0.0]     # Pose válida y cercana al origen


def pose(q1):
    return [q1] + OK[1:]


def nuevo_broker(politica='fifo', **params):
    """Se crea un broker con movimientos muy cortos y se instrumenta la exclusión mutua"""
    Node.PARAMS = dict(politica=politica, duracion_movimiento_s=0.06, pasos_interpolacion=3,
                       archivo_rechazos='')
    Node.PARAMS.update(params)
    br = B.ArmBroker()

    br.trace = []              # goal_id con el que se publicó cada mensaje de /joint_states
    br.max_simultaneos = 0     # Máximo de execute_callback corriendo a la vez
    br.orden_ejecucion = []    # goal_id en el orden en que empezaron
    br._activos = 0
    mover, execute = br.mover, br.execute_callback

    def mover_instrumentado(q):
        with br.lock:
            ej = br.ejecutando
        br.trace.append(ej.goal_id if ej else None)
        mover(q)

    def execute_instrumentado(gh):
        with br.lock:
            br._activos += 1
            br.max_simultaneos = max(br.max_simultaneos, br._activos)
            br.orden_ejecucion.append(bytes(gh.goal_id.uuid).hex())
        try:
            return execute(gh)
        finally:
            with br.lock:
                br._activos -= 1

    br.mover, br.execute_callback = mover_instrumentado, execute_instrumentado
    return br


def arrancar_worker(br):
    """El worker es un timer de 20 ms en ROS 2; aquí lo emula un hilo que lo invoca igual"""
    def bucle():
        while not br._parar.is_set():
            br._worker()
            time.sleep(0.02)
    t = threading.Thread(target=bucle, daemon=True)
    t.start()
    return t


def enviar(br, cliente, q, prioridad=1):
    g = Goal()
    g.joint_positions = q
    g.client_id = cliente
    g.priority = prioridad
    r = br.goal_callback(g)
    if r != GoalResponse.ACCEPT:
        return r, None
    gh = GH(br, g)
    br.handle_accepted_callback(gh)
    return r, gh


def esperar(cond, timeout=10.0):
    fin = time.time() + timeout
    while time.time() < fin:
        if cond():
            return True
        time.sleep(0.005)
    return False


class Base(unittest.TestCase):
    def setUp(self):
        self.brokers = []

    def tearDown(self):
        for br in self.brokers:
            br._parar.set()

    def broker(self, *a, **k):
        br = nuevo_broker(*a, **k)
        self.brokers.append(br)
        return br

    def etiquetas(self, br, envios):
        """goal_id completo -> etiqueta legible (A1, B2...)"""
        return {bytes(gh.goal_id.uuid).hex(): e for e, gh in envios}


@unittest.skipIf(HAY_ROS, 'con ROS 2 real estas pruebas simuladas se omiten')
class TestAdmision(Base):
    def test_goal_valido_aceptado(self):
        br = self.broker()
        r, gh = enviar(br, 'a', OK)
        self.assertEqual(r, GoalResponse.ACCEPT)
        self.assertEqual(br.n_aceptados, 1)
        self.assertEqual(len(br.pendientes), 1)

    def test_encolar_no_ejecuta_ni_publica(self):
        br = self.broker()
        enviar(br, 'a', OK)
        self.assertIsNone(br.ejecutando)
        self.assertEqual(br.pub_joint.msgs, [])

    def test_limite_articular_rechazado_con_motivo(self):
        br = self.broker()
        r, _ = enviar(br, 'a', [3.5, 0, 0, 0, 0, 0])
        self.assertEqual(r, GoalResponse.REJECT)
        motivo = br.get_logger().lineas[-1][1]
        self.assertIn('(limite)', motivo)
        self.assertIn('fuera de rango', motivo)

    def test_workspace_rechazado_con_motivo(self):
        from arm_broker import fk
        import random
        random.seed(3)
        fuera = None
        for _ in range(200000):
            q = [random.uniform(lo, hi) for lo, hi in fk.JOINT_LIMITS]
            if not fk.dentro_del_workspace(q)[0]:
                fuera = q
                break
        self.assertIsNotNone(fuera, 'no encontré una pose fuera del workspace')
        br = self.broker()
        br.q_actual = list(fuera)      # Que el paso articular no sea la causa
        r, _ = enviar(br, 'a', fuera)
        self.assertEqual(r, GoalResponse.REJECT)
        self.assertIn('(workspace)', br.get_logger().lineas[-1][1])

    def test_paso_excesivo_rechazado_con_motivo(self):
        br = self.broker()
        r, _ = enviar(br, 'a', [2.5, 0, 0, 0, 0, 0])      # Desde q_actual = 0
        self.assertEqual(r, GoalResponse.REJECT)
        self.assertIn('(paso)', br.get_logger().lineas[-1][1])

    def test_longitud_y_nan_rechazados(self):
        br = self.broker()
        self.assertEqual(enviar(br, 'a', [0, 0, 0])[0], GoalResponse.REJECT)
        self.assertEqual(enviar(br, 'a', [float('nan')] + [0] * 5)[0], GoalResponse.REJECT)
        self.assertEqual(br.n_rechazados, 2)

    def test_todo_rechazo_queda_en_el_csv_con_motivo(self):
        with tempfile.TemporaryDirectory() as d:
            ruta = os.path.join(d, 'rechazos.csv')
            br = self.broker(archivo_rechazos=ruta)
            enviar(br, 'a', [3.5, 0, 0, 0, 0, 0])
            enviar(br, 'b', [2.5, 0, 0, 0, 0, 0])
            with open(ruta, encoding='utf-8') as f:
                filas = list(csv.DictReader(f))
            self.assertEqual([f['causa'] for f in filas], ['limite', 'paso'])
            self.assertTrue(all(f['motivo'] for f in filas))

    def test_cola_llena(self):
        br = self.broker(cola_max=3)
        for _ in range(3):
            self.assertEqual(enviar(br, 'a', OK)[0], GoalResponse.ACCEPT)
        self.assertEqual(enviar(br, 'a', OK)[0], GoalResponse.REJECT)
        self.assertIn('(cola_llena)', br.get_logger().lineas[-1][1])

    def test_cupo_de_la_cola_bajo_concurrencia(self):
        br = self.broker(cola_max=5)
        respuestas = []

        def uno(i):
            g = Goal()
            g.joint_positions, g.client_id, g.priority = OK, f'c{i % 4}', 1
            r = br.goal_callback(g)
            respuestas.append(r)
            if r == GoalResponse.ACCEPT:
                time.sleep(0.02)                 # Hueco entre goal_callback y handle_accepted
                br.handle_accepted_callback(GH(br, g))
        hilos = [threading.Thread(target=uno, args=(i,)) for i in range(40)]
        [h.start() for h in hilos]
        [h.join() for h in hilos]
        self.assertEqual(sum(r == GoalResponse.ACCEPT for r in respuestas), 5)
        self.assertEqual(len(br.pendientes), 5)
        self.assertEqual(br.reservados, 0)

    def test_uuid_completa(self):
        br = self.broker()
        _, gh = enviar(br, 'a', OK)
        self.assertEqual(br.pendientes[0].goal_id, bytes(gh.goal_id.uuid).hex())
        self.assertEqual(len(br.pendientes[0].goal_id), 32)


@unittest.skipIf(HAY_ROS, 'con ROS 2 real estas pruebas simuladas se omiten')
class TestEjecucion(Base):
    def test_exclusion_mutua_nunca_mas_de_un_goal_ejecutando(self):
        br = self.broker()
        envios = [(f'{c}{i}', enviar(br, c, pose(0.1 * i))[1])
                  for c in 'ABC' for i in (1, 2)]
        arrancar_worker(br)
        for _, gh in envios:
            self.assertTrue(gh.terminado.wait(10))
        self.assertEqual(br.max_simultaneos, 1)
        self.assertNotIn(None, br.trace, 'se publicó en /joint_states sin un goal ejecutando')
        # Cada goal publica en un bloque contiguo: nunca intercalado con otro
        bloques = [g for i, g in enumerate(br.trace) if i == 0 or g != br.trace[i - 1]]
        self.assertEqual(len(bloques), len(set(bloques)))
        self.assertTrue(all(gh.state == 'succeeded' for _, gh in envios))
        self.assertTrue(esperar(lambda: br.n_completados == 6))

    def test_resultado_con_tiempos_y_feedback(self):
        br = self.broker()
        _, g1 = enviar(br, 'a', pose(0.1))
        _, g2 = enviar(br, 'b', pose(0.2))
        arrancar_worker(br)
        self.assertTrue(g2.terminado.wait(10))
        r1, r2 = g1.result_future.result().result, g2.result_future.result().result
        self.assertTrue(r1.success and r2.success)
        self.assertGreater(r2.wait_time_s, r1.wait_time_s)
        self.assertGreater(r2.exec_time_s, 0.05)
        self.assertEqual(g2.feedbacks.count('EXECUTING'), 3)

    def test_paso_se_revalida_al_ejecutar(self):
        br = self.broker()
        r_a, gh_a = enviar(br, 'a', [1.0] + OK[1:])      # Cada uno a 1.0 rad de q_actual = 0
        r_b, gh_b = enviar(br, 'b', [-1.0] + OK[1:])     # ... pero B queda a 2.0 rad de A
        self.assertEqual((r_a, r_b), (GoalResponse.ACCEPT, GoalResponse.ACCEPT))
        arrancar_worker(br)
        self.assertTrue(gh_b.terminado.wait(10))
        self.assertEqual(gh_a.state, 'succeeded')
        self.assertEqual(gh_b.state, 'aborted')
        self.assertIn('al ejecutar', gh_b.result_future.result().result.message)
        self.assertAlmostEqual(br.pub_joint.msgs[-1].position[0], 1.0)   # B no movió el brazo
        # Los contadores son solo de goal_callback: el aborto no los toca
        self.assertEqual((br.n_aceptados, br.n_rechazados), (2, 0))
        self.assertTrue(any('ABORTADO: paso_al_ejecutar' in m
                            for _, m in br.get_logger().lineas))


@unittest.skipIf(HAY_ROS, 'con ROS 2 real estas pruebas simuladas se omiten')
class TestCancelacion(Base):
    def test_cancelado_en_cola_devuelve_resultado(self):
        br = self.broker(duracion_movimiento_s=0.3)
        _, ga = enviar(br, 'a', pose(0.1))
        _, gb = enviar(br, 'b', pose(0.2))
        _, gc = enviar(br, 'c', pose(0.3))
        gb.cancelar()                    # Cancelado mientras esperaba en la cola
        arrancar_worker(br)
        self.assertTrue(gb.terminado.wait(10), 'el cliente se quedaría esperando el Result')
        self.assertTrue(gb.result_future.done())
        r = gb.result_future.result().result
        self.assertFalse(r.success)
        self.assertEqual(r.message, 'cancelado mientras esperaba en cola')
        self.assertEqual(r.exec_time_s, 0.0)
        self.assertGreaterEqual(r.wait_time_s, 0.0)
        self.assertEqual(gb.state, 'canceled')
        self.assertTrue(gc.terminado.wait(10))
        self.assertEqual((ga.state, gc.state), ('succeeded', 'succeeded'))
        self.assertNotIn(bytes(gb.goal_id.uuid).hex(), br.trace)   # No movió el brazo

    def test_cancelado_en_cola_mientras_otro_se_ejecuta(self):
        br = self.broker(duracion_movimiento_s=0.4)
        _, ga = enviar(br, 'a', pose(0.1))
        _, gb = enviar(br, 'b', pose(0.2))
        arrancar_worker(br)
        self.assertTrue(esperar(lambda: br.ejecutando is not None))
        gb.cancelar()
        self.assertTrue(gb.terminado.wait(10))
        self.assertEqual(gb.result_future.result().result.message,
                         'cancelado mientras esperaba en cola')
        self.assertTrue(ga.terminado.wait(10))
        self.assertEqual(ga.state, 'succeeded')

    def test_cancelado_durante_la_ejecucion(self):
        br = self.broker(duracion_movimiento_s=0.6)
        _, ga = enviar(br, 'a', pose(0.1))
        arrancar_worker(br)
        self.assertTrue(esperar(lambda: len(br.pub_joint.msgs) >= 1))
        ga.cancelar()
        self.assertTrue(ga.terminado.wait(10))
        r = ga.result_future.result().result
        self.assertEqual(ga.state, 'canceled')
        self.assertFalse(r.success)
        self.assertIn('durante la ejecución', r.message)
        self.assertTrue(esperar(lambda: br.ejecutando is None))
        _, gb = enviar(br, 'b', pose(0.2))               # Y el broker sigue atendiendo
        self.assertTrue(gb.terminado.wait(10))
        self.assertEqual(gb.state, 'succeeded')

    def test_excepcion_en_atender_no_bloquea_al_cliente_ni_al_worker(self):
        br = self.broker()
        _, ga = enviar(br, 'a', pose(0.1))
        _, gb = enviar(br, 'b', pose(0.2))
        atender = br._atender
        fallos = []

        def atender_roto(pedido):
            if pedido.client_id == 'a' and not fallos:
                fallos.append(pedido)
                raise RuntimeError('falla simulada')     # Antes de llamar a execute()
            return atender(pedido)
        br._atender = atender_roto
        arrancar_worker(br)
        self.assertTrue(ga.terminado.wait(10), 'el cliente se quedaría esperando el Result')
        r = ga.result_future.result().result
        self.assertEqual(ga.state, 'aborted')
        self.assertFalse(r.success)
        self.assertIn('falla simulada', r.message)
        self.assertIs(fallos[0].resultado, r)
        self.assertTrue(fallos[0].fin.is_set())
        self.assertTrue(gb.terminado.wait(10), 'el worker quedó bloqueado')
        self.assertEqual(gb.state, 'succeeded')
        self.assertTrue(esperar(lambda: br.n_completados == 1))


@unittest.skipIf(HAY_ROS, 'con ROS 2 real estas pruebas simuladas se omiten')
class TestPoliticasEnElBroker(Base):
    """Cola con A1 A2 A3 B1 B2 C1 C2 pendientes a la vez, enviados en ese orden"""

    def correr(self, politica):
        br = self.broker(politica)
        envios = []
        for c, n in (('A', 3), ('B', 2), ('C', 2)):
            for i in range(1, n + 1):
                envios.append((f'{c}{i}', enviar(br, c, pose(0.05 * i))[1]))
                time.sleep(0.002)                        # Llegadas en orden estricto
        arrancar_worker(br)
        for _, gh in envios:
            self.assertTrue(gh.terminado.wait(15))
        nombres = self.etiquetas(br, envios)
        return [nombres[g] for g in br.orden_ejecucion], br

    def test_fifo(self):
        orden, br = self.correr('fifo')
        self.assertEqual(orden, ['A1', 'A2', 'A3', 'B1', 'B2', 'C1', 'C2'])
        self.assertEqual(br.max_simultaneos, 1)

    def test_round_robin(self):
        orden, br = self.correr('round_robin')
        self.assertEqual(orden, ['A1', 'B1', 'C1', 'A2', 'B2', 'C2', 'A3'])
        self.assertEqual(br.max_simultaneos, 1)

    def test_los_dos_ordenes_son_distintos(self):
        self.assertNotEqual(self.correr('fifo')[0], self.correr('round_robin')[0])

    def test_con_clientes_asincronos_de_verdad(self):
        """Tres Cliente en modo asíncrono contra el broker: A1 A2 A3 / B1 B2 / C1 C2"""
        for politica, esperado in (
                ('fifo', ['A1', 'A2', 'A3', 'B1', 'B2', 'C1', 'C2']),
                ('round_robin', ['A1', 'B1', 'C1', 'A2', 'B2', 'C2', 'A3'])):
            br = self.broker(politica)
            hilos, clientes = [], []
            for c, n in (('A', 3), ('B', 2), ('C', 2)):
                cli = crear_cliente(br, c, n)
                clientes.append(cli)
                h = threading.Thread(target=cli.correr_asincrono, daemon=True)
                hilos.append(h)
                antes = len(br.pendientes)
                h.start()
                self.assertTrue(esperar(lambda: len(br.pendientes) == antes + n),
                                f'{c} no dejó sus {n} pedidos pendientes')
            self.assertEqual(len(br.pendientes), 7)      # Todo pendiente antes de arrancar
            arrancar_worker(br)
            for h in hilos:
                h.join(15)
                self.assertFalse(h.is_alive(), 'un cliente asíncrono no terminó')
            nombres = {}
            for cli in clientes:
                for i, gh in enumerate(cli.cli.enviados, start=1):
                    nombres[bytes(gh.goal_id.uuid).hex()] = f'{cli.client_id}{i}'
            self.assertEqual([nombres[g] for g in br.orden_ejecucion], esperado, politica)
            self.assertEqual(br.max_simultaneos, 1)
            self.assertTrue(all(c.faltan == 0 for c in clientes))


# 4. Cliente con un ActionClient simulado que habla directo con el broker
class ClienteFalso:
    def __init__(self, br):
        self.br = br
        self.enviados = []

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        fut = Fut()
        r = self.br.goal_callback(goal)
        if r == GoalResponse.ACCEPT:
            gh = GH(self.br, goal)
            self.enviados.append(gh)
            self.br.handle_accepted_callback(gh)
            handle = types.SimpleNamespace(accepted=True,
                                           get_result_async=lambda: gh.result_future)
        else:
            handle = types.SimpleNamespace(accepted=False)
        fut.set_result(handle)
        return fut


def crear_cliente(br, nombre, n_poses):
    Node.PARAMS = dict(client_id=nombre, modo='asincrono', pausa_s=0.0)
    cli = C.Cliente()
    cli.cli = ClienteFalso(br)
    cli.poses = [pose(0.05 * i) for i in range(1, n_poses + 1)]
    return cli


@unittest.skipIf(HAY_ROS, 'con ROS 2 real estas pruebas simuladas se omiten')
class TestCliente(unittest.TestCase):
    def cargar(self, texto):
        with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False, encoding='utf-8') as f:
            f.write(texto)
        try:
            Node.PARAMS = {}
            return C.Cliente().cargar(f.name)
        finally:
            os.unlink(f.name)

    def test_traza_valida_ignora_comentarios_y_vacias(self):
        poses = self.cargar('# q1,q2,q3,q4,q5,q6\n\n0.1,0,0,0,0,0\n  # otro\n0.2,0,0,0,0,0\n')
        self.assertEqual(poses, [[0.1, 0, 0, 0, 0, 0], [0.2, 0, 0, 0, 0, 0]])

    def test_fila_con_menos_de_seis_valores(self):
        with self.assertRaisesRegex(ValueError, 'cada pose debe tener 6 ángulos; llegaron 5'):
            self.cargar('0.1,0,0,0,0\n')

    def test_fila_con_mas_de_seis_valores(self):
        with self.assertRaisesRegex(ValueError, 'cada pose debe tener 6 ángulos; llegaron 7'):
            self.cargar('0.1,0,0,0,0,0,9\n')

    def test_valor_no_numerico_o_nan(self):
        with self.assertRaisesRegex(ValueError, 'no es un número'):
            self.cargar('0.1,0,0,0,0,abc\n')
        with self.assertRaisesRegex(ValueError, 'NaN o infinito'):
            self.cargar('0.1,0,0,0,0,nan\n')

    def test_modo_desconocido(self):
        Node.PARAMS = {'modo': 'rapido'}
        with self.assertRaisesRegex(ValueError, 'modo desconocido'):
            C.Cliente()


# 5. Ningún cliente publica en /joint_states
class TestUnicoPublicador(unittest.TestCase):
    def test_solo_el_broker_crea_publicadores(self):
        """Análisis estático de todo el paquete: no necesita ROS 2 ni simulación"""
        publicadores = {}
        for ruta in glob.glob(os.path.join(RAIZ, 'arm_broker', '*.py')):
            with open(ruta, encoding='utf-8') as f:
                arbol = ast.parse(f.read())
            for n in ast.walk(arbol):
                if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'create_publisher':
                    publicadores.setdefault(os.path.basename(ruta), []).append(
                        [a.value for a in n.args
                         if isinstance(a, ast.Constant) and isinstance(a.value, str)])
        self.assertEqual(list(publicadores), ['broker.py'], publicadores)
        temas = sorted(t for lista in publicadores['broker.py'] for t in lista)
        self.assertEqual(temas, ['/arm/queue_state', '/joint_states'])

    def test_solo_mover_publica_en_joint_states(self):
        with open(os.path.join(RAIZ, 'arm_broker', 'broker.py'), encoding='utf-8') as f:
            fuente = f.read()
        self.assertEqual(fuente.count('self.pub_joint.publish('), 1)
        arbol = ast.parse(fuente)
        for clase in [n for n in arbol.body if isinstance(n, ast.ClassDef)]:
            for f in [n for n in clase.body if isinstance(n, ast.FunctionDef)]:
                if 'pub_joint.publish' in ast.get_source_segment(fuente, f):
                    self.assertEqual(f.name, 'mover')

    def test_el_cliente_no_usa_joint_state(self):
        with open(os.path.join(RAIZ, 'arm_broker', 'cliente.py'), encoding='utf-8') as f:
            fuente = f.read()
        self.assertNotIn('JointState', fuente)
        self.assertNotIn('create_publisher', fuente)


if __name__ == '__main__':
    unittest.main()
