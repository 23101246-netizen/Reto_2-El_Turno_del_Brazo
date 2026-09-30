""" Cliente del broker: cada integrante levanta el suyo — Reto 2 """

import csv
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arm_broker_interfaces.action import MoveArm


# 1. Nodo cliente
class Cliente(Node):
    """Se envían al broker las poses de una traza, una tras otra, y se espera cada resultado"""
    """El cliente NUNCA publica en /joint_states: solo envía goals a la acción move_arm"""

    def __init__(self):
        # Nombre del nodo por defecto; para distinguir a varios clientes se cambia al lanzar:
        # ros2 run arm_broker cliente --ros-args -r __node:=cliente_1 -p client_id:=integrante1
        super().__init__('arm_client')

        # Parámetros de ROS 2: se pasan con --ros-args -p nombre:=valor
        self.declare_parameter('client_id', 'alumno')    # Nombre que aparece en /arm/queue_state
        self.declare_parameter('priority', 1)            # 0..255, mayor número = más urgente
        self.declare_parameter('traza', '')              # CSV de poses; vacío = dos poses de prueba
        self.declare_parameter('repeticiones', 1)        # Vueltas completas a la traza
        self.declare_parameter('pausa_s', 0.5)           # Descanso entre un goal y el siguiente

        self.client_id = self.get_parameter('client_id').value
        self.priority = int(self.get_parameter('priority').value)
        self.repeticiones = int(self.get_parameter('repeticiones').value)
        self.pausa = float(self.get_parameter('pausa_s').value)

        self.cli = ActionClient(self, MoveArm, 'move_arm')
        self.poses = self.cargar(self.get_parameter('traza').value)

    # 2. Carga de la traza
    def cargar(self, ruta):
        """Se leen las poses de un CSV: seis ángulos q1..q6 en rad por fila"""
        """Las filas vacías y las que empiezan con # se ignoran"""
        """Retorna una lista de poses, cada una una lista de 6 floats"""
        if not ruta:
            return [[0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [-0.3, 0.0, 0.0, 0.0, 0.0, 0.0]]
        with open(ruta, newline='') as f:
            filas = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith('#')]
        return [[float(v) for v in fila[:6]] for fila in filas]

    # 3. Envío de goals
    def correr(self):
        """Se envía cada pose de la traza al broker y se espera su resultado antes de la siguiente"""
        """Retorna 0 si terminó, o 1 si el broker no apareció"""
        self.get_logger().info(f'[{self.client_id}] esperando al broker...')
        if not self.cli.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('El broker no aparece. ¿Está corriendo?')
            return 1

        # Se recorre la traza completa tantas veces como indique `repeticiones`
        for vuelta in range(self.repeticiones):
            for i, q in enumerate(self.poses):
                # Se arma el goal con la pose, el nombre y la prioridad de este cliente
                goal = MoveArm.Goal()
                goal.joint_positions = q
                goal.client_id = self.client_id
                goal.priority = self.priority

                # Se envía el goal; el broker responde enseguida si lo acepta o lo rechaza
                t0 = time.time()
                envio = self.cli.send_goal_async(goal, feedback_callback=self.feedback)
                rclpy.spin_until_future_complete(self, envio)
                handle = envio.result()

                # Un goal rechazado lleva su motivo en el log del broker y en rechazos.csv
                if not handle.accepted:
                    self.get_logger().warn(f'[{self.client_id}] pose {i}: RECHAZADA')
                    continue

                # Se espera el resultado: incluye cuánto esperó en la cola y cuánto tardó en ejecutarse
                res_fut = handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_fut)
                r = res_fut.result().result
                self.get_logger().info(
                    f'[{self.client_id}] pose {i}: success={r.success} '
                    f'espera={r.wait_time_s:.2f}s ejec={r.exec_time_s:.2f}s '
                    f'total={time.time() - t0:.2f}s — {r.message}')
                time.sleep(self.pausa)
        return 0

    # 4. Feedback del broker
    def feedback(self, msg):
        """Se muestra el estado del goal: QUEUED con su posición en la cola, o EXECUTING"""
        """Se limita a un mensaje por segundo para no llenar la terminal"""
        f = msg.feedback
        self.get_logger().info(
            f'[{self.client_id}] {f.state} pos={f.queue_position} t={f.elapsed_s:.1f}s',
            throttle_duration_sec=1.0)


# 5. Punto de entrada
def main(args=None):
    """Se inicia ROS 2, se corre el cliente hasta terminar y se sale con su código de retorno"""
    rclpy.init(args=args)
    nodo = Cliente()
    codigo = 0
    try:
        codigo = nodo.correr()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
