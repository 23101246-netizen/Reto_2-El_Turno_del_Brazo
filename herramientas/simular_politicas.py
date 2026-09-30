#!/usr/bin/env python3
""" Predicción de las métricas del ítem 3 con un modelo de cola — ANTES de medir """

"""Se simula un brazo que atiende de a un goal, con tiempo de servicio fijo, usando las MISMAS
clases FIFO y RoundRobin del broker. No mueve nada ni usa ROS 2: sirve para declarar la
predicción del documento de diseño previo antes de tener datos reales"""

"""Uso:
    python3 simular_politicas.py --poses 10 --repeticiones 1 --servicio 3.05
    python3 simular_politicas.py --poses 10 --patron simultaneo

    --clientes   nombre:prioridad, separados por coma (por defecto A:1,B:2,C:3,D:4)
    --poses      poses de la traza (cada cliente envía poses x repeticiones goals)
    --servicio   segundos que el brazo tarda por goal (~ duracion_movimiento_s)
    --patron     escalonado (cada cliente arranca `--escalon` s después del anterior y envía
                 todo de golpe) | simultaneo (todos arrancan a la vez y los goals se intercalan)"""

"""Limitaciones del modelo: el servicio es constante, no hay rechazos ni cancelaciones, y las
esperas son las verdaderas; la medición real (5 Hz) las ve con ~0.2 s de resolución"""

import argparse
import os
import statistics
import sys
import types

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(RAIZ, 'src', 'arm_broker'))
sys.path.insert(0, os.path.join(RAIZ, 'analisis'))

from arm_broker.politicas import POLITICAS, Pedido   # noqa: E402
from metricas import jain, p95                        # noqa: E402

ENTRE_GOALS_S = 0.005    # Separación entre goals consecutivos de un mismo cliente


# 1. Llegadas
def llegadas(clientes, n_goals, patron, escalon):
    """Se generan (instante, cliente, prioridad, número de goal) ordenados por instante"""
    lista = []
    for i, (nombre, prioridad) in enumerate(clientes):
        inicio = i * escalon if patron == 'escalonado' else i * ENTRE_GOALS_S / len(clientes)
        for k in range(n_goals):
            lista.append((inicio + k * ENTRE_GOALS_S, nombre, prioridad, k + 1))
    lista.sort(key=lambda x: (x[0], x[1]))
    return lista


# 2. Simulación de un brazo con una política
def simular(nombre_politica, entradas, servicio):
    """Retorna una lista de dicts con cliente, prioridad, llegada, inicio y espera de cada goal"""
    politica = POLITICAS[nombre_politica]()
    pendientes = []
    hechos = []
    t = 0.0
    i = 0
    while i < len(entradas) or pendientes:
        # Ingresan a la cola todos los goals que ya llegaron
        while i < len(entradas) and entradas[i][0] <= t:
            ins, cliente, prioridad, n = entradas[i]
            gh = types.SimpleNamespace(goal_id=types.SimpleNamespace(uuid=bytes([1]) * 16))
            p = Pedido(gh, cliente, prioridad, [0.0] * 6)
            p.t_llegada, p.etiqueta = ins, f'{cliente}{n}'
            pendientes.append(p)
            i += 1
        if not pendientes:
            t = entradas[i][0]          # Brazo ocioso: se salta hasta la próxima llegada
            continue
        p = pendientes.pop(politica.siguiente(list(pendientes)))
        hechos.append({'goal': p.etiqueta, 'cliente': p.client_id, 'prioridad': p.priority,
                       'llegada': p.t_llegada, 'inicio': t, 'espera': t - p.t_llegada})
        politica.atendido(p)
        t += servicio
    return hechos


# 3. Métricas del modelo (mismas definiciones que analisis/metricas.py)
def metricas(hechos):
    esperas = [h['espera'] for h in hechos]
    por_prioridad = {}
    por_cliente = {}
    for h in hechos:
        por_prioridad.setdefault(h['prioridad'], []).append(h['espera'])
        por_cliente[h['cliente']] = por_cliente.get(h['cliente'], 0) + 1
    inicios = [h['inicio'] for h in hechos]
    mitad = (min(inicios) + max(inicios)) / 2
    en_mitad = {c: 0 for c in por_cliente}
    for h in hechos:
        if h['inicio'] <= mitad:
            en_mitad[h['cliente']] += 1
    peor = min(por_prioridad)
    return {
        'media': statistics.mean(esperas), 'p95': p95(esperas), 'max': max(esperas),
        'por_prioridad': {p: (statistics.mean(v), p95(v), max(v))
                          for p, v in sorted(por_prioridad.items())},
        'inanicion': max(por_prioridad[peor]), 'jain': jain(list(por_cliente.values())),
        'jain_mitad': jain(list(en_mitad.values())), 'en_mitad': en_mitad,
        'orden': [h['goal'] for h in hechos],
    }


# 4. Punto de entrada
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clientes', default='A:1,B:2,C:3,D:4')
    ap.add_argument('--poses', type=int, default=10)
    ap.add_argument('--repeticiones', type=int, default=1)
    ap.add_argument('--servicio', type=float, default=3.05)
    ap.add_argument('--patron', choices=('escalonado', 'simultaneo'), default='escalonado')
    ap.add_argument('--escalon', type=float, default=0.5)
    ap.add_argument('--orden', action='store_true', help='imprime el orden de atención')
    args = ap.parse_args()

    clientes = [(c.split(':')[0], int(c.split(':')[1])) for c in args.clientes.split(',')]
    n_goals = args.poses * args.repeticiones
    entradas = llegadas(clientes, n_goals, args.patron, args.escalon)

    print(f'{len(clientes)} clientes x {n_goals} goals = {len(entradas)} goals · servicio '
          f'{args.servicio}s · patrón {args.patron} · duración total '
          f'{len(entradas) * args.servicio:.0f}s')
    for politica in ('fifo', 'round_robin'):
        m = metricas(simular(politica, entradas, args.servicio))
        print(f'\n=== {politica}')
        print(f'  espera media {m["media"]:.1f}s · p95 {m["p95"]:.1f}s · máxima {m["max"]:.1f}s')
        for pr, (media, q95, mx) in m['por_prioridad'].items():
            print(f'    prioridad {pr}: media={media:6.1f}s  p95={q95:6.1f}s  máx={mx:6.1f}s')
        print(f'  índice de inanición {m["inanicion"]:.1f}s')
        print(f'  Jain al final {m["jain"]:.3f} · a mitad {m["jain_mitad"]:.3f} {m["en_mitad"]}')
        if args.orden:
            print('  orden:', ' '.join(m['orden']))


if __name__ == '__main__':
    main()
