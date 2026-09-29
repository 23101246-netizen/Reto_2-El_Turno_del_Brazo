"""Cinemática directa del JetCobot — Ítem 1 del Reto del Brazo 2.

Modelo basado en:
- la estructura de fk.py entregada por el curso;
- la tabla DH definida por el equipo/profesor;
- convención DH estándar.

Unidades:
- q: radianes
- a, d: milímetros
- salida fk(q): (x, y, z) en milímetros
"""

import math


JOINT_NAMES = [
    '1_Joint',
    '2_Joint',
    '3_Joint',
    '4_Joint',
    '5_Joint',
    '6_Joint',
]


# ---------------------------------------------------------------------------
# TABLA DH
#
# Formato por fila:
#   (alpha_i [rad], a_i [mm], d_i [mm], theta_offset_i [rad])
#
# Tabla usada:
#
# i     theta                 d [mm]    a [mm]    alpha
# 1     q1                    134.75       0       +90°
# 2     q2 - 90°                0       -110        0°
# 3     q3                       0        -96        0°
# 4     q4 - 90°               63.4        0       +90°
# 5     q5 + 90°              75.55        0       -90°
# 6     q6                      50          0         0°
# ---------------------------------------------------------------------------

DH = [
    ( math.pi / 2,    0.0, 134.75,           0.0),  # J1
    (         0.0, -110.0,   0.00, -math.pi / 2),  # J2
    (         0.0,  -96.0,   0.00,           0.0),  # J3
    ( math.pi / 2,    0.0,  63.40, -math.pi / 2),  # J4
    (-math.pi / 2,    0.0,  75.55,  math.pi / 2),  # J5
    (         0.0,    0.0,  50.00,           0.0),  # J6
]


# Límites articulares entregados en el archivo base del curso.
# Todos en radianes.
JOINT_LIMITS = [
    (-2.93, 2.93),
    (-2.36, 2.36),
    (-2.53, 2.53),
    (-2.58, 2.58),
    (-2.93, 2.93),
    (-3.14, 3.14),
]


# Filtro geométrico simple utilizado por el broker.
ALCANCE_MIN_MM = 80.0
ALCANCE_MAX_MM = 480.0


def _t(alpha, a, d, theta):
    """Matriz homogénea DH estándar de una articulación.

    A_i = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    """
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
    """Multiplica dos matrices 4x4."""
    return [
        [
            sum(A[i][k] * B[k][j] for k in range(4))
            for j in range(4)
        ]
        for i in range(4)
    ]


def fk_matriz(q):
    """Retorna T_0_6, la matriz homogénea base -> efector.

    Parameters
    ----------
    q : iterable de 6 floats
        Ángulos articulares [q1,...,q6] en radianes.
    """
    if len(q) != 6:
        raise ValueError(
            f'fk_matriz(q) requiere 6 ángulos; llegaron {len(q)}'
        )

    # Matriz identidad 4x4.
    T = [
        [1.0 if i == j else 0.0 for j in range(4)]
        for i in range(4)
    ]

    # T_0_6 = A1 * A2 * ... * A6
    for (alpha, a, d, offset), theta in zip(DH, q):
        A_i = _t(alpha, a, d, theta + offset)
        T = _mul(T, A_i)

    return T


def fk(q):
    """Calcula la posición cartesiana del efector final.

    Parameters
    ----------
    q : iterable de 6 floats
        Ángulos articulares en radianes.

    Returns
    -------
    tuple
        (x, y, z) en milímetros.
    """
    T = fk_matriz(q)

    x = T[0][3]
    y = T[1][3]
    z = T[2][3]

    return x, y, z


def dentro_de_limites(q):
    """Comprueba que las seis articulaciones estén dentro de sus límites."""
    if len(q) != 6:
        return False, f'se esperaban 6 ángulos, llegaron {len(q)}'

    for i, (valor, (lo, hi)) in enumerate(zip(q, JOINT_LIMITS)):
        if not lo <= valor <= hi:
            return (
                False,
                f'{JOINT_NAMES[i]} fuera de rango: '
                f'{valor:.3f} rad, límite [{lo}, {hi}]'
            )

    return True, ''


def dentro_del_workspace(q):
    """Filtro básico del workspace usando la posición calculada por FK."""
    x, y, z = fk(q)
    r = math.sqrt(x * x + y * y + z * z)

    if r > ALCANCE_MAX_MM:
        return (
            False,
            f'efector a {r:.0f} mm de la base, '
            f'máximo {ALCANCE_MAX_MM:.0f}'
        )

    if r < ALCANCE_MIN_MM:
        return False, f'efector a {r:.0f} mm de la base, demasiado cerca'

    if z < 0.0:
        return False, f'z = {z:.0f} mm: el efector quedaría bajo la base'

    return True, ''


def paso_articular(q_desde, q_hasta):
    """Máxima variación articular entre dos configuraciones, en radianes."""
    if len(q_desde) != 6 or len(q_hasta) != 6:
        raise ValueError('paso_articular requiere dos vectores de 6 articulaciones')

    return max(
        abs(b - a)
        for a, b in zip(q_desde, q_hasta)
    )
