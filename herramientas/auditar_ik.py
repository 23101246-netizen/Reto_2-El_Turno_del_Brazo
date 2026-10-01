#!/usr/bin/env python3
""" Auditoría de la cinemática inversa del firmware con la FK propia — Ítem 4 del Reto 2 """

"""La cinemática inversa la resuelve el firmware (send_coords); el equipo NO escribe solver
Este script la audita: pide un objetivo cartesiano, lee el q que el brazo realmente adoptó,
le aplica la FK propia (fk.py) y mide el error contra lo solicitado:

    objetivo (x, y, z) -> send_coords() -> el firmware resuelve la IK -> el brazo adopta q
        -> get_angles() [°] -> q [rad] -> fk.fk(q) -> error = |FK(q) - objetivo|

La evidencia principal es FK(q_real) frente al objetivo. get_coords() se guarda solo como dato
de apoyo: si FK(q_real) y get_coords() coinciden pero ambos están lejos del objetivo, el firmware
no llegó; si FK(q_real) y get_coords() difieren de forma constante, hay un desfase de marco o de
la tabla DH (ver docs/item4_auditoria_ik.md)"""

"""Uso (en el Jetson, con el puerto serie libre y el espacio despejado alrededor del brazo):

    python3 auditar_ik.py --plantilla          # imprime la tabla vacía; no necesita el robot
    python3 auditar_ik.py --solo-leer          # no mueve: compara FK(q) con get_coords() donde está
    python3 auditar_ik.py --x X --y Y --z Z --rx RX --ry RY --rz RZ \\
                          --confirmo-espacio-despejado --guardar

No hay objetivo por defecto: el punto físico seguro sobre el tablero se define en la sesión"""

import argparse
import csv
import math
import os
import sys
import time

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(RAIZ, 'src', 'arm_broker'))
from arm_broker import fk                                        # noqa: E402

# 1. Parámetros por defecto
PUERTO = '/dev/ttyUSB0'
BAUD = 1000000
VELOCIDAD = 30          # 0..100 en pymycobot; baja a propósito
ESPERA_S = 5.0          # Segundos de espera a que termine el movimiento
RUTA_CSV = os.path.join(RAIZ, 'evidencias', 'item4', 'auditoria_ik.csv')

# Las primeras 16 columnas son las acordadas; el resto es información de apoyo
COLUMNAS = [
    'x_obj', 'y_obj', 'z_obj',
    'q1_deg', 'q2_deg', 'q3_deg', 'q4_deg', 'q5_deg', 'q6_deg',
    'x_fk', 'y_fk', 'z_fk',
    'x_robot', 'y_robot', 'z_robot',
    'error_mm',
    'rx_obj', 'ry_obj', 'rz_obj', 'velocidad',
    'ex_mm', 'ey_mm', 'ez_mm', 'dif_fk_robot_mm',
]


# 2. Matemática del error (se prueba sin robot)
def error_cartesiano(objetivo, obtenido):
    """Se calcula e = sqrt((X_FK-X_obj)² + (Y_FK-Y_obj)² + (Z_FK-Z_obj)²) en mm"""
    """Entrada: objetivo y obtenido = (x, y, z) en mm"""
    if len(objetivo) != 3 or len(obtenido) != 3:
        raise ValueError('error_cartesiano requiere dos puntos (x, y, z)')
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(objetivo, obtenido)))


def errores_por_eje(objetivo, obtenido):
    """Se devuelve (ex, ey, ez) = obtenido - objetivo, en mm, con signo"""
    return tuple(a - b for a, b in zip(obtenido, objetivo))


def grados_a_radianes(q_grados):
    """get_angles() entrega grados; fk.py trabaja en radianes"""
    return [math.radians(v) for v in q_grados]


def radianes_a_grados(q_rad):
    return [math.degrees(v) for v in q_rad]


# 3. Lectura del brazo
def leer_estado(brazo):
    """Se lee q [°] con get_angles() y la pose con get_coords(); si el brazo no responde, se avisa"""
    """pymycobot devuelve una lista de números, o una lista vacía / un entero (-1) si falla"""
    q_deg = brazo.get_angles()
    coords = brazo.get_coords()
    if not isinstance(q_deg, (list, tuple)) or len(q_deg) != 6:
        raise RuntimeError(f'el brazo no respondió a get_angles(): {q_deg!r}')
    if not isinstance(coords, (list, tuple)) or len(coords) < 3:
        raise RuntimeError(f'el brazo no respondió a get_coords(): {coords!r}')
    return list(q_deg), list(coords)


# 4. Registro de una auditoría
def auditar(q_deg, coords_robot, objetivo_xyz, orientacion=None, velocidad=None):
    """Se aplica la FK propia al q realmente adoptado y se compara con el objetivo"""
    """Retorna un dict con todo lo que se guarda como evidencia"""
    q_rad = grados_a_radianes(q_deg)
    xyz_fk = tuple(fk.fk(q_rad))
    xyz_robot = tuple(coords_robot[:3])
    objetivo = tuple(objetivo_xyz)
    return {
        'objetivo': objetivo,
        'orientacion': tuple(orientacion) if orientacion is not None else None,
        'velocidad': velocidad,
        'q_deg': list(q_deg),
        'q_rad': q_rad,
        'xyz_fk': xyz_fk,
        'xyz_robot': xyz_robot,
        'error_mm': error_cartesiano(objetivo, xyz_fk),
        'error_ejes': errores_por_eje(objetivo, xyz_fk),
        'dif_fk_robot_mm': error_cartesiano(xyz_robot, xyz_fk),
    }


def registro_a_fila(r):
    """Se ordena un registro según COLUMNAS"""
    ori = r['orientacion'] or ('', '', '')
    fila = (list(r['objetivo']) + list(r['q_deg']) + list(r['xyz_fk']) + list(r['xyz_robot'])
            + [r['error_mm']] + list(ori) + [r['velocidad'] if r['velocidad'] is not None else '']
            + list(r['error_ejes']) + [r['dif_fk_robot_mm']])
    return [f'{v:.4f}' if isinstance(v, float) else v for v in fila]


def guardar_csv(registro, ruta=RUTA_CSV):
    """Se agrega la fila a la evidencia; si el archivo no existe o está vacío, primero va el encabezado"""
    os.makedirs(os.path.dirname(os.path.abspath(ruta)), exist_ok=True)
    nuevo = not os.path.isfile(ruta) or os.path.getsize(ruta) == 0
    with open(ruta, 'a', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(COLUMNAS)
        w.writerow(registro_a_fila(registro))


# 5. Tabla de evidencia
def _fmt(valores, decimales=1):
    return '(' + ', '.join(f'{v:.{decimales}f}' for v in valores) + ')'


def tabla_evidencia(r=None):
    """Se arma la tabla Variable / Valor del ítem 4; sin registro, todo queda «pendiente»"""
    p = 'pendiente'
    if r is None:
        filas = [
            ('Objetivo solicitado X, Y, Z [mm]', p), ('Orientación utilizada [°]', p),
            ('Velocidad', p), ('get_angles() [°]', p), ('q_real [rad]', p),
            ('get_coords()', p), ('FK(q_real) [mm]', p), ('Error X [mm]', p),
            ('Error Y [mm]', p), ('Error Z [mm]', p), ('Error cartesiano total [mm]', p),
        ]
    else:
        ori = _fmt(r['orientacion']) if r['orientacion'] else 'no registrada'
        vel = r['velocidad'] if r['velocidad'] is not None else 'no registrada'
        filas = [
            ('Objetivo solicitado X, Y, Z [mm]', _fmt(r['objetivo'])),
            ('Orientación utilizada [°]', ori), ('Velocidad', vel),
            ('get_angles() [°]', _fmt(r['q_deg'], 2)), ('q_real [rad]', _fmt(r['q_rad'], 4)),
            ('get_coords()', _fmt(r['xyz_robot'])), ('FK(q_real) [mm]', _fmt(r['xyz_fk'])),
            ('Error X [mm]', f"{r['error_ejes'][0]:+.2f}"),
            ('Error Y [mm]', f"{r['error_ejes'][1]:+.2f}"),
            ('Error Z [mm]', f"{r['error_ejes'][2]:+.2f}"),
            ('Error cartesiano total [mm]', f"{r['error_mm']:.2f}"),
        ]
    ancho = max(len(k) for k, _ in filas)
    return '\n'.join([f'{"Variable":<{ancho}}  Valor', '-' * (ancho + 2 + 30)]
                     + [f'{k:<{ancho}}  {v}' for k, v in filas])


# 6. Objetivo y flujo con el brazo
def advertencias_objetivo(xyz):
    """Se revisan cosas evidentes del objetivo; son avisos, no un solver ni una prueba de alcanzabilidad"""
    if not all(math.isfinite(v) for v in xyz):
        raise ValueError(f'el objetivo contiene valores no numéricos: {xyz}')
    avisos = []
    r = math.sqrt(sum(v * v for v in xyz))
    if not fk.ALCANCE_MIN_MM <= r <= fk.ALCANCE_MAX_MM:
        avisos.append(f'‖objetivo‖ = {r:.0f} mm, fuera de {fk.ALCANCE_MIN_MM:.0f}-'
                      f'{fk.ALCANCE_MAX_MM:.0f} mm medidos desde la base')
    return avisos


def ejecutar(brazo, objetivo_xyz, orientacion=None, velocidad=VELOCIDAD, modo=0,
             espera_s=ESPERA_S, mover=True):
    """Se pide el objetivo al firmware (si mover), se espera, se lee q y se audita con la FK propia"""
    """Entrada: brazo = objeto con send_coords / get_angles / get_coords (un MyCobot)
    modo: 0 = movimiento angular, 1 = lineal (parámetro de send_coords)"""
    if mover:
        if orientacion is None:
            raise ValueError('para mover el brazo hace falta la orientación (rx, ry, rz)')
        brazo.send_coords(list(objetivo_xyz) + list(orientacion), velocidad, modo)
        time.sleep(espera_s)
    q_deg, coords = leer_estado(brazo)
    return auditar(q_deg, coords, objetivo_xyz, orientacion, velocidad)


# 7. Punto de entrada
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--z', type=float)
    ap.add_argument('--rx', type=float)
    ap.add_argument('--ry', type=float)
    ap.add_argument('--rz', type=float)
    ap.add_argument('--velocidad', type=int, default=VELOCIDAD)
    ap.add_argument('--modo', type=int, choices=(0, 1), default=0)
    ap.add_argument('--espera', type=float, default=ESPERA_S)
    ap.add_argument('--puerto', default=PUERTO)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--csv', default=RUTA_CSV)
    ap.add_argument('--plantilla', action='store_true', help='imprime la tabla vacía y sale')
    ap.add_argument('--solo-leer', action='store_true', help='no mueve el brazo')
    ap.add_argument('--guardar', action='store_true', help='agrega la fila a la evidencia CSV')
    ap.add_argument('--confirmo-espacio-despejado', action='store_true')
    args = ap.parse_args(argv)

    if args.plantilla:
        print(tabla_evidencia())
        return 0

    objetivo = (args.x, args.y, args.z)
    orientacion = (args.rx, args.ry, args.rz)
    hay_objetivo = None not in objetivo
    hay_orientacion = None not in orientacion

    # Sin objetivo físico definido no se mueve nada
    if not args.solo_leer:
        if not hay_objetivo or not hay_orientacion:
            print('Aún no se definió el objetivo físico: faltan --x --y --z y --rx --ry --rz.\n'
                  'No hay objetivo por defecto; se fija en la sesión, con el espacio despejado.',
                  file=sys.stderr)
            return 2
        if not args.confirmo_espacio_despejado:
            print('Va a moverse el brazo. Avise en voz alta, despeje el espacio y agregue '
                  '--confirmo-espacio-despejado.', file=sys.stderr)
            return 2
        for aviso in advertencias_objetivo(objetivo):
            print('AVISO:', aviso, file=sys.stderr)

    try:
        from pymycobot.mycobot import MyCobot
    except ImportError:
        try:
            from pymycobot import MyCobot
        except ImportError:
            print('Falta pymycobot (pip install pymycobot); se corre en el Jetson.', file=sys.stderr)
            return 1

    if args.solo_leer and not hay_objetivo and args.guardar:
        print('El diagnóstico sin objetivo no se guarda como evidencia del ítem 4.', file=sys.stderr)
        return 2

    brazo = MyCobot(args.puerto, args.baud)
    try:
        if args.solo_leer and not hay_objetivo:
            # Sin objetivo se compara contra get_coords(): es un diagnóstico FK vs firmware,
            # NO la auditoría del ítem 4
            _, coords = leer_estado(brazo)
            objetivo = tuple(coords[:3])
            print('Modo diagnóstico: objetivo = get_coords(). Esto no es la auditoría de la IK.')
        r = ejecutar(brazo, objetivo, orientacion if hay_orientacion else None, args.velocidad,
                     args.modo, args.espera, mover=not args.solo_leer)
    except RuntimeError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1

    print(tabla_evidencia(r))
    print(f'\ndiferencia FK(q_real) vs get_coords(): {r["dif_fk_robot_mm"]:.2f} mm '
          f'(dato de apoyo)')
    print(f'criterio del reto: error ≤ 10 mm -> {"CUMPLE" if r["error_mm"] <= 10 else "NO CUMPLE"}')
    if args.guardar:
        guardar_csv(r, args.csv)
        print(f'fila agregada a {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
