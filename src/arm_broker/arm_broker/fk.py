"""Cinemática directa del JetCobot — Ítem 1 del Reto 2."""

import math

JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]

DH = [
    ( math.pi / 2,    0.0, 131.22,           0.0),
    (         0.0, -110.4,   0.00, -math.pi / 2),
    (         0.0,  -96.0,   0.00,           0.0),
    ( math.pi / 2,    0.0,  63.40, -math.pi / 2),
    (-math.pi / 2,    0.0,  75.05,  math.pi / 2),
    (         0.0,    0.0,  45.60,           0.0),
]

JOINT_LIMITS = [
    (-2.87979, 2.87979),
    (-2.87979, 2.87979),
    (-2.87979, 2.87979),
    (-2.87979, 2.87979),
    (-2.87979, 2.87979),
    (-3.05433, 3.05433),
]

ALCANCE_MIN_MM = 80.0
ALCANCE_MAX_MM = 480.0
RADIO_NOMINAL_MM = 280.0


def _t(alpha, a, d, theta):
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)

    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,       sa,       ca,      d],
        [0.0,      0.0,      0.0,    1.0],
    ]


def _mul(A, B):
    return [
        [
            sum(A[i][k] * B[k][j] for k in range(4))
            for j in range(4)
        ]
        for i in range(4)
    ]


def fk_matriz(q):
    if len(q) != 6:
        raise ValueError(f'fk_matriz(q) requiere 6 articulaciones; se recibieron {len(q)}')

    T = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    for (alpha, a, d, offset), theta in zip(DH, q):
        theta_dh = theta + offset
        A_i = _t(alpha, a, d, theta_dh)
        T = _mul(T, A_i)

    return T


def fk(q):
    T = fk_matriz(q)
    return T[0][3], T[1][3], T[2][3]


def dentro_de_limites(q):
    if len(q) != 6:
        return False, f'Se esperaban 6 ángulos, llegaron {len(q)}'

    for i, (valor, (lo, hi)) in enumerate(zip(q, JOINT_LIMITS)):
        if not lo <= valor <= hi:
            return False, f'{JOINT_NAMES[i]} fuera de rango: {valor:.3f} rad'

    return True, ''


def dentro_del_workspace(q):
    x, y, z = fk(q)
    r = math.sqrt(x * x + y * y + z * z)

    if r > ALCANCE_MAX_MM:
        return False, f'Efector a {r:.0f} mm de la base, máximo {ALCANCE_MAX_MM:.0f} mm'

    if r < ALCANCE_MIN_MM:
        return False, f'Efector a {r:.0f} mm de la base, demasiado cerca'

    if z < 0.0:
        return False, f'z = {z:.0f} mm: el efector quedaría bajo la base'

    return True, ''


def paso_articular(q_desde, q_hasta):
    if len(q_desde) != 6 or len(q_hasta) != 6:
        raise ValueError('paso_articular requiere dos vectores de 6 articulaciones')

    return max(abs(b - a) for a, b in zip(q_desde, q_hasta))
