"""Pruebas de las políticas de cola. No necesitan ROS 2.

    cd src/arm_broker && python3 -m unittest discover -s test -v
"""
import itertools
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from arm_broker.politicas import FIFO, POLITICAS, Pedido, RoundRobin  # noqa: E402

_ids = itertools.count(1)


def pedido(cliente, t, prioridad=1):
    gh = types.SimpleNamespace(goal_id=types.SimpleNamespace(uuid=bytes([next(_ids)] * 16)))
    p = Pedido(gh, cliente, prioridad, [0.0] * 6)
    p.t_llegada = t
    return p


def atender(politica, cola):
    """Simula al worker: elige, saca de la cola, avisa a la política."""
    orden = []
    while cola:
        i = politica.siguiente(cola)
        p = cola.pop(i)
        politica.atendido(p)
        orden.append(p)
    return orden


class TestFIFO(unittest.TestCase):
    def test_cola_vacia(self):
        self.assertIsNone(FIFO().siguiente([]))

    def test_orden_de_llegada(self):
        cola = [pedido('a', 3), pedido('b', 1), pedido('c', 2)]
        orden = atender(FIFO(), cola)
        self.assertEqual([p.client_id for p in orden], ['b', 'c', 'a'])

    def test_ignora_la_prioridad(self):
        cola = [pedido('a', 1, prioridad=0), pedido('b', 2, prioridad=255)]
        self.assertEqual(FIFO().siguiente(cola), 0)


class TestRoundRobin(unittest.TestCase):
    def test_cola_vacia(self):
        self.assertIsNone(RoundRobin().siguiente([]))

    def test_alterna_entre_clientes(self):
        cola = [pedido('a', 1), pedido('a', 2), pedido('a', 3),
                pedido('b', 4), pedido('c', 5)]
        orden = atender(RoundRobin(), cola)
        self.assertEqual([p.client_id for p in orden], ['a', 'b', 'c', 'a', 'a'])

    def test_un_cliente_no_acapara(self):
        # 'a' llegó con 6 pedidos antes que los demás; FIFO los atendería todos primero.
        cola = [pedido('a', t) for t in range(1, 7)] + [pedido('b', 7), pedido('c', 8)]
        orden = [p.client_id for p in atender(RoundRobin(), cola)]
        self.assertLessEqual(orden.index('b'), 1)
        self.assertLessEqual(orden.index('c'), 2)

    def test_dentro_de_un_cliente_el_mas_antiguo(self):
        cola = [pedido('a', 5), pedido('a', 2), pedido('a', 9)]
        rr = RoundRobin()
        self.assertEqual(cola[rr.siguiente(cola)].t_llegada, 2)

    def test_cliente_sin_pedidos_no_consume_turno(self):
        rr = RoundRobin()
        cola = [pedido('a', 1), pedido('b', 2)]
        atender(rr, cola)
        # 'a' y 'b' ya se vieron; ahora solo llega 'b': se atiende de inmediato
        nueva = [pedido('b', 3)]
        self.assertEqual(rr.siguiente(nueva), 0)

    def test_ignora_la_prioridad(self):
        cola = [pedido('a', 1, prioridad=0), pedido('b', 2, prioridad=255)]
        orden = atender(RoundRobin(), cola)
        self.assertEqual([p.client_id for p in orden], ['a', 'b'])

    def test_no_inanicion_con_cuatro_clientes(self):
        cola = [pedido(c, i * 4 + j) for i in range(5) for j, c in enumerate('abcd')]
        orden = [p.client_id for p in atender(RoundRobin(), cola)]
        for vuelta in range(5):
            self.assertEqual(sorted(orden[vuelta * 4:vuelta * 4 + 4]), list('abcd'))


class TestRegistro(unittest.TestCase):
    def test_nombres(self):
        self.assertEqual(set(POLITICAS), {'fifo', 'round_robin'})
        for nombre, clase in POLITICAS.items():
            self.assertEqual(clase().nombre, nombre)


if __name__ == '__main__':
    unittest.main()
