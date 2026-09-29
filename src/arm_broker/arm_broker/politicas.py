"""Políticas de cola — ítems 2 y 3."""

import threading
import time


class Pedido:
    def __init__(self, goal_handle, client_id, priority, joint_positions):
        self.goal_handle = goal_handle
        self.goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        self.client_id = client_id
        self.priority = int(priority)
        self.joint_positions = list(joint_positions)
        self.t_llegada = time.time()
        self.t_inicio_ejec = None
        self.fin = threading.Event()
        self.resultado = None

    @property
    def espera_s(self):
        fin = self.t_inicio_ejec if self.t_inicio_ejec else time.time()
        return fin - self.t_llegada

    def __repr__(self):
        return f'<{self.client_id} p{self.priority} {self.goal_id}>'


class Politica:
    nombre = 'base'

    def siguiente(self, pendientes):
        raise NotImplementedError

    def atendido(self, pedido):
        pass


# ============================== IMPLEMENTAR · ítems 2 y 3 =========================
# FIFO es obligatoria. La segunda la eligen ustedes y la justifican en el README:
# prioridad estática, prioridad con envejecimiento, o round-robin entre clientes.
#
# siguiente(pendientes) devuelve el ÍNDICE del pedido a atender, o None.
# Cada Pedido trae: client_id, priority, t_llegada y espera_s.

class FIFO(Politica):
    nombre = 'fifo'

    def siguiente(self, pendientes):
        raise NotImplementedError('El que llegó primero')


class SegundaPolitica(Politica):
    nombre = 'prioridad'

    def siguiente(self, pendientes):
        raise NotImplementedError('La política que elijan')

# =================================================================================


POLITICAS = {
    'fifo': FIFO,
    'prioridad': SegundaPolitica,
}
