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
# FIFO (obligatoria) y Round Robin entre clientes (la política elegida por el equipo).
#
# siguiente(pendientes) devuelve el ÍNDICE del pedido a atender, o None.
# Cada Pedido trae: client_id, priority, t_llegada y espera_s.
#
# Ambas son funciones puras del estado de la cola: no modifican `pendientes`
# (eso lo hace el worker del broker, bajo su lock) y solo el worker las llama.

class FIFO(Politica):
    """Atiende en orden estricto de llegada, sin mirar cliente ni prioridad."""
    nombre = 'fifo'

    def siguiente(self, pendientes):
        if not pendientes:
            return None
        return min(range(len(pendientes)), key=lambda i: pendientes[i].t_llegada)


class RoundRobin(Politica):
    """Reparte el turno entre clientes por rotación; ignora la prioridad.

    Recuerda el orden en que apareció cada cliente y quién fue el último atendido.
    En cada decisión recorre los clientes en ese orden circular, empezando por el
    siguiente al último atendido, y toma el primero que tenga algo pendiente; dentro
    de un cliente, su pedido más antiguo. Un cliente sin pedidos no consume turno.

    Efecto: ningún cliente espera más de (n_clientes - 1) atenciones ajenas por
    turno propio, así que no hay inanición. A cambio, un cliente que envía muchos
    pedidos no puede acaparar el brazo, y la prioridad numérica no adelanta a nadie.
    """
    nombre = 'round_robin'

    def __init__(self):
        self.clientes = []      # orden de primera aparición
        self.ultimo = None      # cliente atendido más recientemente

    def siguiente(self, pendientes):
        if not pendientes:
            return None

        for p in pendientes:
            if p.client_id not in self.clientes:
                self.clientes.append(p.client_id)

        n = len(self.clientes)
        if self.ultimo in self.clientes:
            inicio = (self.clientes.index(self.ultimo) + 1) % n
        else:
            inicio = 0

        for k in range(n):
            cliente = self.clientes[(inicio + k) % n]
            propios = [i for i, p in enumerate(pendientes) if p.client_id == cliente]
            if propios:
                return min(propios, key=lambda i: pendientes[i].t_llegada)
        return None

    def atendido(self, pedido):
        self.ultimo = pedido.client_id

# =================================================================================


POLITICAS = {
    'fifo': FIFO,
    'round_robin': RoundRobin,
}
