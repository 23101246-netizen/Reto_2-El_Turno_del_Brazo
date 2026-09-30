""" Broker de acceso exclusivo al JetCobot — Ítem 2 del Reto 2 """


import csv
import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_broker_interfaces.action import MoveArm
from arm_broker_interfaces.msg import QueueState

from arm_broker import fk
from arm_broker.politicas import POLITICAS, Pedido


# 1. Nodo del broker
class ArmBroker(Node):
    """Es el ÚNICO nodo que publica en /joint_states; los clientes solo envían goals"""
    """Un goal pasa por tres etapas: admisión (goal_callback), encolado
    (handle_accepted_callback) y ejecución exclusiva (worker -> execute_callback)"""

    def __init__(self):
        super().__init__('arm_broker')

        # Parámetros de ROS 2: se pasan con --ros-args -p nombre:=valor
        self.declare_parameter('politica', 'fifo')                # fifo | round_robin
        self.declare_parameter('cola_max', 20)                    # Pedidos pendientes admitidos
        self.declare_parameter('paso_max_rad', 1.2)               # Salto articular máximo por movimiento
        self.declare_parameter('duracion_movimiento_s', 3.0)      # Duración de cada movimiento
        self.declare_parameter('pasos_interpolacion', 10)         # Mensajes /joint_states por movimiento
        self.declare_parameter('archivo_rechazos', 'rechazos.csv')  # Vacío desactiva el registro

        # Se elige la política de cola según el parámetro
        nombre = self.get_parameter('politica').value
        if nombre not in POLITICAS:
            raise RuntimeError(f'política desconocida: {nombre}. Hay {list(POLITICAS)}')
        self.politica = POLITICAS[nombre]()

        self.cola_max = int(self.get_parameter('cola_max').value)
        self.paso_max = float(self.get_parameter('paso_max_rad').value)
        self.duracion = float(self.get_parameter('duracion_movimiento_s').value)
        self.pasos = max(1, int(self.get_parameter('pasos_interpolacion').value))
        self.archivo_rechazos = str(self.get_parameter('archivo_rechazos').value)

        # Reentrant permite aceptar goals mientras otro se ejecuta; MutuallyExclusive serializa al worker
        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()

        # Estado compartido: todo acceso a lo que sigue se hace bajo self.lock
        self.lock = threading.Lock()
        self.lock_rechazos = threading.Lock()   # Solo para escribir el CSV de rechazos
        self.pendientes = []                    # Pedidos en cola, en orden de llegada
        self.por_goal_id = {}                   # goal_id -> Pedido, para execute_callback
        self.ejecutando = None                  # Pedido en marcha, o None
        self.q_actual = [0.0] * 6               # Última pose publicada en /joint_states
        self.reservados = 0     # Aceptados en goal_callback que aún no llegan a self.pendientes
        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0
        self._parar = threading.Event()

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_cola = self.create_publisher(QueueState, '/arm/queue_state', 10)

        self.servidor = ActionServer(
            self,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.grupo_entrada,
        )

        self.create_timer(0.2, self.publicar_estado_cola,
                          callback_group=self.grupo_worker)

        # El worker es el único que saca pedidos de la cola y los manda a ejecutar
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'arm_broker listo · política={self.politica.nombre} · '
            f'cola_max={self.cola_max} · único publicador de /joint_states')

    # 2. Admisión de goals
    def goal_callback(self, goal_request):
        """Se decide si un goal entra: barata e inmediata, acepta o rechaza y nunca ejecuta"""
        """Se rechaza con motivo explícito si el objetivo está fuera de límites articulares,
        fuera del workspace, con paso articular excesivo o si la cola está llena"""
        """No se espera al brazo ni se toca /joint_states: solo se hacen cuentas con fk.py"""
        """Retorna GoalResponse.ACCEPT o GoalResponse.REJECT"""
        q = list(goal_request.joint_positions)
        cliente = goal_request.client_id or '?'

        causa, motivo = self._validar(q)

        with self.lock:
            if causa is None:
                # Comprobar el cupo y reservarlo es UNA sola operación bajo el lock: si
                # fueran dos, varios clientes simultáneos verían el mismo hueco libre y
                # la cola superaría cola_max. La reserva se cancela en handle_accepted
                if len(self.pendientes) + self.reservados >= self.cola_max:
                    causa = 'cola_llena'
                    motivo = f'cola llena ({self.cola_max} pedidos pendientes)'
                else:
                    self.reservados += 1
                    self.n_aceptados += 1
                    return GoalResponse.ACCEPT
            self.n_rechazados += 1

        # Todo rechazo queda en el log y en el CSV, siempre con su causa y su motivo
        self.get_logger().warn(
            f'RECHAZADO [{cliente} p{goal_request.priority}] ({causa}): {motivo}')
        self._registrar_rechazo(goal_request, causa, motivo)
        return GoalResponse.REJECT

    # 3. Validación del objetivo con la FK
    def _validar(self, q):
        """Se comprueba que el objetivo q sea admisible: límites, workspace y paso articular"""
        """No se mira el cupo de la cola: eso se decide (y se reserva) en goal_callback"""
        """Retorna (None, '') si es admisible, o (causa, motivo) con causa en
        'limite', 'workspace' o 'paso'"""
        # Valores no numéricos: los comparadores de fk.py no los detectarían bien
        if any(not math.isfinite(v) for v in q):
            return 'limite', 'el objetivo contiene valores no numéricos (NaN/inf)'

        # Límites articulares: también cubre que lleguen exactamente 6 ángulos, y debe
        # ir primero porque la FK del workspace exige 6 articulaciones
        ok, motivo = fk.dentro_de_limites(q)
        if not ok:
            return 'limite', motivo

        # Workspace: posición del efector calculada con la FK
        ok, motivo = fk.dentro_del_workspace(q)
        if not ok:
            return 'workspace', motivo

        # Paso articular máximo desde la pose actual
        with self.lock:
            q_desde = list(self.q_actual)
        paso = fk.paso_articular(q_desde, q)
        if paso > self.paso_max:
            return 'paso', (
                f'paso articular de {paso:.2f} rad desde la pose actual, '
                f'máximo {self.paso_max:.2f}')

        return None, ''

    # 4. Registro de rechazos
    def _registrar_rechazo(self, goal_request, causa, motivo):
        """Se agrega el rechazo a un CSV: evidencia del ítem 2 (todo rechazo lleva motivo)"""
        """Columnas: t_unix, client_id, priority, causa, motivo, joint_positions"""
        if not self.archivo_rechazos:
            return
        try:
            # Varios goals pueden rechazarse a la vez: la escritura se serializa
            with self.lock_rechazos:
                nuevo = not os.path.exists(self.archivo_rechazos)
                with open(self.archivo_rechazos, 'a', newline='', encoding='utf-8') as f:
                    w = csv.writer(f)
                    if nuevo:
                        w.writerow(['t_unix', 'client_id', 'priority', 'causa',
                                    'motivo', 'joint_positions'])
                    w.writerow([f'{time.time():.3f}', goal_request.client_id,
                                goal_request.priority, causa, motivo,
                                ' '.join(f'{v:.4f}' for v in goal_request.joint_positions)])
        except OSError as e:
            self.get_logger().error(f'no pude escribir {self.archivo_rechazos}: {e}')

    # 5. Encolado
    def handle_accepted_callback(self, goal_handle):
        """Se encola el goal aceptado: AQUÍ NO SE EJECUTA NADA ni se publica en /joint_states"""
        """Se crea el Pedido y se deja en self.pendientes (e indexado por goal_id) bajo self.lock"""
        """El worker es el único que decide cuándo le toca"""
        goal = goal_handle.request
        pedido = Pedido(goal_handle, goal.client_id, goal.priority, goal.joint_positions)
        with self.lock:
            self.reservados = max(0, self.reservados - 1)   # La reserva pasa a ser un pedido real
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
        self.get_logger().info(
            f'ENCOLADO {pedido!r} · pendientes={len(self.pendientes)}')

    # 6. Worker único
    def _worker(self):
        """Se desencola y ejecuta de a un pedido: es el único que decide a quién le toca"""
        """Corre en su propio hilo. Nunca hay dos pedidos en marcha: el siguiente solo se elige
        cuando pedido.fin avisa que el anterior terminó, y esa espera es la exclusión mutua"""
        while not self._parar.is_set():
            pedido = None
            self._purgar_cancelados()

            # Si el brazo está libre, la política elige a quién le toca y se saca de la cola
            with self.lock:
                if self.ejecutando is None and self.pendientes:
                    indice = self.politica.siguiente(list(self.pendientes))
                    if indice is not None:
                        pedido = self.pendientes.pop(indice)
                        self.ejecutando = pedido
                        pedido.t_inicio_ejec = time.time()

            # Nada que hacer: se espera un instante antes de volver a mirar la cola
            if pedido is None:
                self._parar.wait(0.02)
                continue

            try:
                self._atender(pedido)
            except Exception as e:  # El worker no puede morir: se caería el broker
                self.get_logger().error(f'error atendiendo {pedido!r}: {e!r}')
            finally:
                # Pase lo que pase, el brazo se libera y la política se entera
                with self.lock:
                    self.por_goal_id.pop(pedido.goal_id, None)
                    self.ejecutando = None
                    if pedido.resultado is not None and pedido.resultado.success:
                        self.n_completados += 1
                self.politica.atendido(pedido)

    # 7. Descarte de pedidos cancelados
    def _purgar_cancelados(self):
        """Se sacan de la cola los pedidos cancelados mientras esperaban; no mueven el brazo"""
        with self.lock:
            cancelados = [p for p in self.pendientes if p.goal_handle.is_cancel_requested]
            for p in cancelados:
                self.pendientes.remove(p)
                self.por_goal_id.pop(p.goal_id, None)
        for p in cancelados:
            p.goal_handle.canceled()
            self.get_logger().info(f'DESCARTADO (cancelado en cola) {p!r}')

    # 8. Atención de un pedido
    def _atender(self, pedido):
        """Se lanza un pedido y se espera a que termine; solo lo llama el worker"""
        goal_handle = pedido.goal_handle

        # Si lo cancelaron justo antes de empezar, se descarta sin mover el brazo
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info(f'DESCARTADO (cancelado en cola) {pedido!r}')
            return

        self.get_logger().info(
            f'EJECUTANDO {pedido!r} · esperó {pedido.espera_s:.2f}s')
        goal_handle.execute()   # El executor corre execute_callback en otro hilo

        # Se espera a que termine ANTES de sacar el siguiente: aquí está la exclusión mutua
        while not pedido.fin.wait(0.1):
            if self._parar.is_set():
                return

    # 9. Ejecución con interpolación
    def execute_callback(self, goal_handle):
        """Se ejecuta UN pedido; lo llama el worker (vía goal_handle.execute()), nunca handle_accepted"""
        """Se interpola desde self.q_actual hasta el destino en self.pasos pasos, publicando con
        self.mover() y mandando feedback en cada uno; la cancelación se revisa en cada paso"""
        """Pase lo que pase, al final se hace pedido.fin.set() para liberar al worker"""
        """Retorna el Result con success, message, wait_time_s y exec_time_s"""
        # Se busca el Pedido que corresponde a este goal
        goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        with self.lock:
            pedido = self.por_goal_id.get(goal_id)

        resultado = MoveArm.Result()
        if pedido is None:
            goal_handle.abort()
            resultado.success = False
            resultado.message = 'el goal no estaba encolado en el broker'
            return resultado

        try:
            with self.lock:
                origen = list(self.q_actual)
            destino = pedido.joint_positions

            # Segunda validación del paso, ahora que le toca: en goal_callback se midió
            # contra la pose de entonces, y los pedidos de delante pudieron moverla.
            # Nadie más publica mientras corre este callback, así que q_actual es exacta
            paso = fk.paso_articular(origen, destino)
            if paso > self.paso_max:
                motivo = (f'paso articular de {paso:.2f} rad desde la pose actual al '
                          f'ejecutar, máximo {self.paso_max:.2f}')
                self.get_logger().warn(
                    f'ABORTADO antes de mover [{pedido.client_id} p{pedido.priority}] '
                    f'(paso_al_ejecutar): {motivo}')
                self._registrar_rechazo(goal_handle.request, 'paso_al_ejecutar', motivo)
                with self.lock:   # Deja de contar como aceptado: pasa a rechazado
                    self.n_aceptados -= 1
                    self.n_rechazados += 1
                resultado.wait_time_s = float(pedido.espera_s)
                resultado.success = False
                resultado.message = f'rechazado al ejecutar: {motivo}'
                goal_handle.abort()
                return resultado

            # Tiempos del pedido: cuánto esperó en la cola y cuánto dura cada paso
            t_ini = pedido.t_inicio_ejec or time.time()
            espera = t_ini - pedido.t_llegada
            dt = self.duracion / self.pasos
            cancelado = False

            # Se recorre la interpolación lineal en el espacio articular
            for k in range(1, self.pasos + 1):
                # Se comprueba la cancelación antes de cada paso
                if goal_handle.is_cancel_requested or self._parar.is_set():
                    cancelado = True
                    break

                if k == self.pasos:
                    q = list(destino)   # El último paso cae exactamente en el destino
                else:
                    f = k / self.pasos
                    q = [a + (b - a) * f for a, b in zip(origen, destino)]
                self.mover(q)

                # Se avisa al cliente del avance
                fb = MoveArm.Feedback()
                fb.state = 'EXECUTING'
                fb.queue_position = 0
                fb.elapsed_s = time.time() - t_ini
                goal_handle.publish_feedback(fb)

                self._parar.wait(dt)

            # Se cierra el goal como cancelado o exitoso, con sus tiempos
            resultado.wait_time_s = float(espera)
            resultado.exec_time_s = float(time.time() - t_ini)

            if cancelado:
                goal_handle.canceled()
                resultado.success = False
                resultado.message = 'cancelado durante la ejecución; el brazo quedó donde estaba'
            else:
                goal_handle.succeed()
                resultado.success = True
                resultado.message = 'ok'
            return resultado
        except Exception as e:
            self.get_logger().error(f'fallo ejecutando {pedido!r}: {e!r}')
            goal_handle.abort()
            resultado.success = False
            resultado.message = f'error en la ejecución: {e!r}'
            return resultado
        finally:
            pedido.resultado = resultado
            pedido.fin.set()   # Si no, el worker se queda esperando para siempre

    # 10. Cancelación
    def cancel_callback(self, goal_handle):
        """Se acepta toda cancelación; el worker la atiende al descartar o entre pasos"""
        return CancelResponse.ACCEPT

    # 11. Publicación en /joint_states
    def mover(self, q):
        """Se publica la pose q en /joint_states: solo lo llama execute_callback"""
        """Entrada: q = [q1,q2,q3,q4,q5,q6] en radianes rad"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = fk.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub_joint.publish(msg)
        with self.lock:
            self.q_actual = list(q)

    # 12. Estado de la cola a 5 Hz
    def publicar_estado_cola(self):
        """Se publica /arm/queue_state (lo llama un timer de 0.2 s), visible para todos los clientes"""
        """A cada goal en cola se le manda además su feedback QUEUED con su posición"""
        # Se toma una foto de la cola bajo el lock y se publica fuera de él
        msg = QueueState()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            ej = self.ejecutando
            msg.executing_client = ej.client_id if ej else ''
            msg.executing_goal_id = ej.goal_id if ej else ''
            msg.executing_elapsed_s = (time.time() - ej.t_inicio_ejec) if ej and ej.t_inicio_ejec else 0.0
            msg.queue_length = len(self.pendientes)
            msg.queued_goal_ids = [p.goal_id for p in self.pendientes]
            msg.queued_clients = [p.client_id for p in self.pendientes]
            msg.queued_priorities = [min(255, max(0, p.priority)) for p in self.pendientes]
            msg.queued_wait_s = [p.espera_s for p in self.pendientes]
            msg.total_accepted = self.n_aceptados
            msg.total_rejected = self.n_rechazados
            msg.total_completed = self.n_completados
            cola = list(self.pendientes)
        self.pub_cola.publish(msg)

        for posicion, p in enumerate(cola, start=1):
            try:
                fb = MoveArm.Feedback()
                fb.state = 'QUEUED'
                fb.queue_position = posicion
                fb.elapsed_s = p.espera_s
                p.goal_handle.publish_feedback(fb)
            except Exception:
                pass

    # 13. Cierre del nodo
    def destroy_node(self):
        """Se avisa al worker y al execute_callback que deben terminar"""
        self._parar.set()
        return super().destroy_node()


# 14. Punto de entrada
def main(args=None):
    """Se inicia ROS 2 y se hace girar el broker con un executor multihilo"""
    """Multihilo hace falta para aceptar goals mientras otro se ejecuta"""
    rclpy.init(args=args)
    nodo = ArmBroker()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
