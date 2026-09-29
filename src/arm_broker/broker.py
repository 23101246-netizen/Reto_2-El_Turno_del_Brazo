"""arm_broker — el único nodo que publica en /joint_states."""

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


class ArmBroker(Node):

    def __init__(self):
        super().__init__('arm_broker')

        self.declare_parameter('politica', 'fifo')
        self.declare_parameter('tau_envejecimiento_s', 8.0)
        self.declare_parameter('cola_max', 20)
        self.declare_parameter('paso_max_rad', 1.2)
        self.declare_parameter('duracion_movimiento_s', 3.0)
        self.declare_parameter('pasos_interpolacion', 10)

        nombre = self.get_parameter('politica').value
        if nombre not in POLITICAS:
            raise RuntimeError(f'política desconocida: {nombre}. Hay {list(POLITICAS)}')
        clase = POLITICAS[nombre]
        if nombre == 'prioridad':
            self.politica = clase(self.get_parameter('tau_envejecimiento_s').value)
        else:
            self.politica = clase()

        self.cola_max = int(self.get_parameter('cola_max').value)
        self.paso_max = float(self.get_parameter('paso_max_rad').value)
        self.duracion = float(self.get_parameter('duracion_movimiento_s').value)
        self.pasos = max(1, int(self.get_parameter('pasos_interpolacion').value))

        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()

        self.lock = threading.Lock()
        self.pendientes = []
        self.por_goal_id = {}
        self.ejecutando = None
        self.q_actual = [0.0] * 6
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

        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'arm_broker listo · política={self.politica.nombre} · '
            f'cola_max={self.cola_max} · único publicador de /joint_states')

    # ------------------------------------------------------------ admisión
    def goal_callback(self, goal_request):
        q = list(goal_request.joint_positions)
        cliente = goal_request.client_id or '?'

        ok, motivo = fk.dentro_de_limites(q)
        if not ok:
            return self._rechazar(cliente, f'límites articulares: {motivo}')

        ok, motivo = fk.dentro_del_workspace(q)
        if not ok:
            return self._rechazar(cliente, f'workspace: {motivo}')

        with self.lock:
            desde = list(self.q_actual)
            en_cola = len(self.pendientes)
        paso = fk.paso_articular(desde, q)
        if paso > self.paso_max:
            return self._rechazar(
                cliente, f'paso articular excesivo: {paso:.2f} rad, máximo {self.paso_max:.2f}')

        if en_cola >= self.cola_max:
            return self._rechazar(cliente, f'cola llena ({en_cola}/{self.cola_max})')

        with self.lock:
            self.n_aceptados += 1
        x, y, z = fk.fk(q)
        self.get_logger().info(
            f'ACEPTADO  {cliente}  ->  ({x:.0f}, {y:.0f}, {z:.0f}) mm')
        return GoalResponse.ACCEPT

    def _rechazar(self, cliente, motivo):
        with self.lock:
            self.n_rechazados += 1
        self.get_logger().warn(f'RECHAZADO {cliente}: {motivo}')
        self._ultimo_rechazo = motivo
        return GoalResponse.REJECT

    # ------------------------------------------------------------- encolar
    def handle_accepted_callback(self, goal_handle):
        pedido = Pedido(goal_handle,
                        goal_handle.request.client_id,
                        goal_handle.request.priority,
                        goal_handle.request.joint_positions)
        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
            posicion = len(self.pendientes)
        self.get_logger().info(
            f'ENCOLADO  {pedido.client_id}  posición {posicion}  (no se ejecuta aquí)')

    # -------------------------------------------------------------- worker
    def _worker(self):
        while not self._parar.is_set():
            pedido = None
            with self.lock:
                if self.ejecutando is None and self.pendientes:
                    idx = self.politica.siguiente(self.pendientes)
                    if idx is not None:
                        pedido = self.pendientes.pop(idx)
                        if pedido.goal_handle.is_cancel_requested:
                            self.por_goal_id.pop(pedido.goal_id, None)
                            pedido = None
                        else:
                            self.ejecutando = pedido
                            pedido.t_inicio_ejec = time.time()
            if pedido is None:
                time.sleep(0.02)
                continue

            pedido.goal_handle.execute()
            pedido.fin.wait()

            with self.lock:
                self.ejecutando = None
                self.por_goal_id.pop(pedido.goal_id, None)
                self.n_completados += 1
            self.politica.atendido(pedido)

    # ------------------------------------------------------------ ejecutar
    def execute_callback(self, goal_handle):
        gid = bytes(goal_handle.goal_id.uuid).hex()[:12]
        with self.lock:
            pedido = self.por_goal_id.get(gid)
        resultado = MoveArm.Result()

        if pedido is None:
            goal_handle.abort()
            resultado.success = False
            resultado.message = 'pedido desconocido'
            return resultado

        try:
            destino = pedido.joint_positions
            with self.lock:
                desde = list(self.q_actual)
            t0 = time.time()
            dt = self.duracion / self.pasos

            for paso in range(1, self.pasos + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    resultado.success = False
                    resultado.message = f'cancelado en el paso {paso}/{self.pasos}'
                    resultado.wait_time_s = pedido.espera_s
                    resultado.exec_time_s = time.time() - t0
                    return resultado

                frac = paso / self.pasos
                q = [a + (b - a) * frac for a, b in zip(desde, destino)]
                self.mover(q)

                fb = MoveArm.Feedback()
                fb.state = 'EXECUTING'
                fb.queue_position = 0
                fb.elapsed_s = time.time() - t0
                goal_handle.publish_feedback(fb)
                time.sleep(dt)

            goal_handle.succeed()
            x, y, z = fk.fk(destino)
            resultado.success = True
            resultado.message = f'llegó a ({x:.0f}, {y:.0f}, {z:.0f}) mm'
            resultado.wait_time_s = pedido.espera_s
            resultado.exec_time_s = time.time() - t0
            self.get_logger().info(
                f'COMPLETADO {pedido.client_id}  espera={resultado.wait_time_s:.2f}s '
                f'ejec={resultado.exec_time_s:.2f}s')
            return resultado
        finally:
            pedido.fin.set()

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # ----------------------------------------------------------- publicar
    def mover(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = fk.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub_joint.publish(msg)
        with self.lock:
            self.q_actual = list(q)

    def publicar_estado_cola(self):
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

    def destroy_node(self):
        self._parar.set()
        return super().destroy_node()


def main(args=None):
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
