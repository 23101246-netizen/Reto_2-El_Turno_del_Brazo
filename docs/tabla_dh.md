# Documento de diseño previo — Tabla DH del JetCobot (Ítem 1)

> **Estado:** predicciones congeladas y validación del ítem 1 hecha (secciones 5 y 6). Quedan por
> completar los campos `[ ... ]` y la firma.

| Campo | Valor |
|---|---|
| Reto | RB-2 · El Turno del Brazo |
| Equipo n.º | `[ ]` (ROS_DOMAIN_ID = 42 + n.º) |
| Integrantes | `[ ]`, `[ ]`, `[ ]`, `[ ]` |
| Fecha del borrador | `[ ]` |
| Fuente de la tabla | Manual del JetCobot (Yahboom): parámetros DH indicados en el manual |
| Implementación | `src/arm_broker/arm_broker/fk.py` (`DH`, `fk_matriz`, `fk`) |

## 1. Convención

Denavit-Hartenberg **estándar** (Craig/Siciliano-Spong). Cada eslabón:

```
A_i = Rot_z(θ_i) · Trans_z(d_i) · Trans_x(a_i) · Rot_x(α_i)
```

```
        | cosθ  -sinθ·cosα   sinθ·sinα   a·cosθ |
A_i  =  | sinθ   cosθ·cosα  -cosθ·sinα   a·sinθ |
        |  0        sinα         cosα        d    |
        |  0         0            0          1    |
```

Cadena completa: `T_0_6 = A1 · A2 · A3 · A4 · A5 · A6`.
La posición del efector es la última columna: `(x, y, z) = T_0_6[0:3, 3]`.

Unidades: longitudes en **mm**, ángulos en **rad** dentro del código (la tabla usa grados).
El ángulo DH es `θ_i = q_i + offset_i`, donde `q_i` es la lectura articular del robot.

## 2. Tabla DH (tomada del manual del JetCobot)

| i | θ_i | d_i [mm] | a_i [mm] | α_i |
|:-:|:---|---:|---:|---:|
| 1 | q1 | 134.75 | 0 | +90° |
| 2 | q2 − 90° | 0 | −110 | 0° |
| 3 | q3 | 0 | −96 | 0° |
| 4 | q4 − 90° | 63.4 | 0 | +90° |
| 5 | q5 + 90° | 75.55 | 0 | −90° |
| 6 | q6 | 50 | 0 | 0° |

Correspondencia con el código (`fk.py`, columnas `alpha, a, d, offset_theta`):

```python
DH = [
    ( math.pi / 2,    0.0, 134.75,           0.0),   # J1
    (         0.0, -110.0,   0.00, -math.pi / 2),    # J2
    (         0.0,  -96.0,   0.00,           0.0),   # J3
    ( math.pi / 2,    0.0,  63.40, -math.pi / 2),    # J4
    (-math.pi / 2,    0.0,  75.55,  math.pi / 2),    # J5
    (         0.0,    0.0,  50.00,           0.0),   # J6
]
```

### Puntos a confirmar antes de firmar

- [ ] **d5 = 75.55 mm.** Valor tomado del manual del JetCobot y usado en `fk.py`; confirmar
      que tabla, esquema y código coinciden con el manual (una diferencia de 0.5 mm sería
      irrelevante frente al criterio de 10 mm, pero el documento debe ser consistente).
- [ ] **α4 = +90°.** Valor del manual, implementado en `fk.py`; confirmar contra la tabla del manual.
- [ ] **Origen del marco {0}.** Definir con precisión el punto físico (eje de J1 a nivel de la
      base) desde donde se mide con regla.
- [ ] **Punto del efector.** Definir qué punto de la pinza/flange representa `T_0_6` (con
      `d6 = 50 mm` la FK apunta a 50 mm del flange de J6 a lo largo de su eje) y medir a ese punto.

## 3. Esquema de marcos (a completar con el dibujo)

Insertar aquí el diagrama de ejes `z_i`, `x_i` por articulación (el del manual del JetCobot,
redibujado). Ayuda de lectura de la cadena, de la base al efector:

```
{0} base ── d1 = 134.75 ─▶ J1 (giro vertical)
   └─ a2 = −110 ─▶ J2 (hombro)
        └─ a3 = −96 ─▶ J3 (codo)
             └─ d4 = 63.4 ─▶ J4
                  └─ d5 = 75.55 ─▶ J5
                       └─ d6 = 50 ─▶ J6 → efector
```

Pose cero (q = 0): el brazo queda estirado hacia arriba; la FK da
`(50.0, −63.4, 416.3) mm`, donde `x = d6` y `y = −d4` (coherente con el esquema).

## 4. Límites y espacio de trabajo usados por la admisión (Ítem 2)

| Articulación | Mínimo [rad] | Máximo [rad] |
|---|---:|---:|
| J1 | −2.93 | 2.93 |
| J2 | −2.36 | 2.36 |
| J3 | −2.53 | 2.53 |
| J4 | −2.58 | 2.58 |
| J5 | −2.93 | 2.93 |
| J6 | −3.14 | 3.14 |

- Alcance: `80 mm ≤ ‖(x, y, z)‖ ≤ 480 mm` y `z ≥ 0`.
- [ ] Contrastar estos límites con la documentación de Yahboom/pymycobot antes de usarlos como
      criterio de rechazo.

## 5. Predicciones (declaradas ANTES de medir)

Se probaron 4 poses en el robot; para el ítem 1 se usan **las 3 primeras**: `cero`, `ready` y
`girada` (la cuarta, `baja`, queda fuera). Predicción calculada con `fk.fk(q)` con el `q` comandado:

| Pose | q comandado [rad] | x_pred [mm] | y_pred [mm] | z_pred [mm] |
|---|---|---:|---:|---:|
| `cero` | [0, 0, 0, 0, 0, 0] | 50.00 | −63.40 | 416.30 |
| `ready` | [0, −0.5, 0.5, 0, 0.5, 0] | 96.62 | −39.43 | 402.83 |
| `girada` | [0.6, −0.4, 0.4, 0, 0.3, 0] | 102.23 | 11.03 | 407.62 |

Congeladas en `evidencias/item1/predicciones_antes_de_medir.txt`, commit `a0cbc35`
(2026-09-30 20:10, hora de Lima), **anterior** al registro de la validación (`84e9f1a`, 21:01).

## 6. Resultados de la medición

Datos en `evidencias/item1/validacion_fk.txt`. Para cada pose se lee el `q` que el brazo realmente
adoptó (`get_angles()`, que no coincide exactamente con el comandado), se calcula `FK(q_real)` y se
compara con la posición que reporta el robot (`get_coords()`). Criterio: error ≤ 10 mm.

| Pose | FK(q_real) [mm] | Robot, `get_coords()` [mm] | Error [mm] | ¿≤ 10 mm? |
|---|---|---|---:|:-:|
| `cero` | (55.9, −62.6, 414.6) | (54.4, −63.2, 409.1) | **5.8** | Sí |
| `ready` | (102.1, −38.6, 400.9) | (100.9, −40.5, 395.4) | **5.9** | Sí |
| `girada` | (108.4, 14.6, 404.4) | (108.2, 11.9, 399.0) | **5.9** | Sí |

Error medio de las 3 poses: 5.9 mm · error máximo: 5.9 mm. **Las tres cumplen el criterio de ≤ 10 mm.**
(La cuarta pose probada, `baja`, dio 6.0 mm y no se cuenta.)

**Análisis del error.** La diferencia `FK(q_real) − Robot` es casi la misma en las tres poses
(≈ +1 mm en x, +0.6 a +2.7 mm en y, **≈ +5.5 mm en z**): un error constante, no uno que crezca con la
distancia, lo que es compatible con un desfase de marco o de herramienta (por ejemplo `d1`, `d5` o `d6`) y
no con un error en los `a_i`. Está dentro del criterio, pero es la primera cosa a revisar si hiciera falta
bajar el error.

**Nota sobre qué se compara.** El brazo no llega exactamente al `q` comandado. Si en lugar de `FK(q_real)`
se comparara la **predicción declarada** (con el `q` comandado) contra el robot, el error sería
8.4 mm (`cero`), 8.6 mm (`ready`) y 10.5 mm (`girada`). Por eso la validación se hace con `q_real`, que
es lo que el brazo realmente adoptó; conviene tenerlo presente en la sustentación.

## 7. Firma

Firmado antes de ejecutar las mediciones:
`[Nombre 1]` · `[Nombre 2]` · `[Nombre 3]` · `[Nombre 4]` — Fecha/hora: `[ ]`

---

*Este documento se completa luego con: diagrama de secuencia (Ítem 2) y la predicción del p95
por política (Ítem 3), que forman parte del mismo «documento de diseño previo».*
