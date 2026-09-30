""" Políticas de cola FIFO y Round Robin — Ítems 2 y 3 del Reto 2 """

import threading
import time


# 1. Pedido: un goal aceptado que espera su turno
class Pedido:
    """Se guarda un goal aceptado junto con los tiempos que usa el broker"""
    """Parametros: goal_handle -> manejador del goal en el ActionServer
                   client_id -> nombre del cliente que lo envió
                   priority -> prioridad 0..255, mayor número = más urgente
                   joint_positions -> objetivo articular q1..q6 en rad"""
    def __init__(self, goal_handle, client_id, priority, joint_positions):
        self.goal_handle = goal_handle
        self.goal_id = bytes(goal_handle.goal_id.uuid).hex()[:12]
        self.client_id = client_id
        self.priority = int(priority)
        self.joint_positions = list(joint_positions)
        self.t_llegada = time.time()    # Instante en que entró a la cola
        self.t_inicio_ejec = None       # Instante en que el worker lo sacó de la cola
        self.fin = threading.Event()    # El worker espera aquí a que termine de ejecutarse
        self.resultado = None           # Result que devuelve execute_callback

    @property
    def espera_s(self):
        """Se calcula cuánto lleva esperando (o cuánto esperó, si ya empezó), en segundos"""
        fin = self.t_inicio_ejec if self.t_inicio_ejec else time.time()
        return fin - self.t_llegada

    def __repr__(self):
        return f'<{self.client_id} p{self.priority} {self.goal_id}>'


# 2. Política base
class Politica:
    """Se define la interfaz que cumplen todas las políticas de cola"""
    nombre = 'base'

    def siguiente(self, pendientes):
        """Se elige a quién le toca; retorna el índice en pendientes, o None si está vacía"""
        raise NotImplementedError

    def atendido(self, pedido):
        """Se avisa a la política que un pedido terminó, por si necesita recordarlo"""
        pass


# 3. Política FIFO
# Es la política obligatoria: se atiende en orden estricto de llegada.
# siguiente(pendientes) devuelve el ÍNDICE del pedido a atender, o None.
# Cada Pedido trae: client_id, priority, t_llegada y espera_s.
# Las políticas no modifican `pendientes`: eso lo hace el worker del broker, bajo su lock.
class FIFO(Politica):
    """Se atiende primero al pedido que llegó primero, sin mirar cliente ni prioridad"""
    nombre = 'fifo'

    def siguiente(self, pendientes):
        """Entrada: pendientes = lista de Pedido
        Salida: índice del pedido con menor t_llegada, o None si no hay pedidos
        """
        if not pendientes:
            return None
        return min(range(len(pendientes)), key=lambda i: pendientes[i].t_llegada)


# 4. Política Round Robin entre clientes
# Es la segunda política: el turno rota entre clientes y la prioridad numérica se ignora.
class RoundRobin(Politica):
    """Se reparte el turno entre clientes por rotación circular"""
    """Se recuerda el orden en que apareció cada cliente y quién fue el último atendido"""
    """En cada decisión se recorren los clientes en ese orden, desde el siguiente al último
    atendido, y se toma el primero que tenga algo pendiente: su pedido más antiguo"""
    """Un cliente sin pedidos no consume turno"""
    """Efecto: ningún cliente espera más de (n_clientes - 1) atenciones ajenas por turno
    propio, así que no hay inanición; a cambio, la prioridad no adelanta a nadie"""
    nombre = 'round_robin'

    def __init__(self):
        self.clientes = []      # Orden de primera aparición
        self.ultimo = None      # Cliente atendido más recientemente

    def siguiente(self, pendientes):
        """Entrada: pendientes = lista de Pedido
        Salida: índice del pedido a atender, o None si no hay pedidos
        """
        if not pendientes:
            return None

        # Se registran los clientes nuevos al final de la rotación
        for p in pendientes:
            if p.client_id not in self.clientes:
                self.clientes.append(p.client_id)

        # La búsqueda empieza en el cliente siguiente al último atendido
        n = len(self.clientes)
        if self.ultimo in self.clientes:
            inicio = (self.clientes.index(self.ultimo) + 1) % n
        else:
            inicio = 0

        # Se recorre la rotación completa y se toma el primer cliente con pedidos
        for k in range(n):
            cliente = self.clientes[(inicio + k) % n]
            propios = [i for i, p in enumerate(pendientes) if p.client_id == cliente]
            if propios:
                return min(propios, key=lambda i: pendientes[i].t_llegada)
        return None

    def atendido(self, pedido):
        """Se recuerda al cliente atendido: la próxima búsqueda empieza después de él"""
        self.ultimo = pedido.client_id


# 5. Registro de políticas disponibles
# La clave es el valor del parámetro `politica` del broker.

POLITICAS = {
    'fifo': FIFO,
    'round_robin': RoundRobin,
}
