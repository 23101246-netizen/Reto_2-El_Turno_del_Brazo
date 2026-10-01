# Documento de diseño previo — Tabla DH del JetCobot (Ítem 1)

> **Estado: BORRADOR.** Completar los campos `[ ... ]` y firmar antes de ejecutar las
> mediciones. La predicción de cada pose se congela con un commit **anterior** a medir.

| Campo | Valor |
|---|---|
| Reto | RB-2 · El Turno del Brazo |
| Equipo n.º | `[ ]` (ROS_DOMAIN_ID = 42 + n.º) |
| Integrantes | `[ ]`, `[ ]`, `[ ]`, `[ ]` |
| Fecha del borrador | `[ ]` |
| Fuente de la tabla | Deducción en pizarra por el equipo (foto adjunta en `docs/`, si se sube) |
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

## 2. Tabla DH (deducida en pizarra)

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

- [ ] **d5: 75.55 vs 75.05 mm.** El esquema de la pizarra rotula un eslabón como 75.05 y la
      tabla dice 75.55. El código usa 75.55. La diferencia (0.5 mm) es irrelevante frente al
      criterio de 10 mm, pero el documento debe ser consistente: elegir un valor y corregir
      tabla, esquema y `fk.py` juntos.
- [ ] **α4 = +90°.** En la foto el valor de la fila 4 se lee «10». Se asume +90° porque es lo
      que implementa `fk.py` y lo que da una geometría coherente; confirmar con el esquema.
- [ ] **Origen del marco {0}.** Definir con precisión el punto físico (eje de J1 a nivel de la
      base) desde donde se mide con regla.
- [ ] **Punto del efector.** Definir qué punto de la pinza/flange representa `T_0_6` (con
      `d6 = 50 mm` la FK apunta a 50 mm del flange de J6 a lo largo de su eje) y medir a ese punto.

## 3. Esquema de marcos (a completar con el dibujo)

Insertar aquí el diagrama de ejes `z_i`, `x_i` por articulación (el de la pizarra, pasado en
limpio). Ayuda de lectura de la cadena, de la base al efector:

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

## 5. Predicciones (declarar ANTES de medir)

Elegir **3 poses** cuya posición sea fácil de medir con regla o cinta (evitar la pose cero:
el efector queda a >400 mm de altura). Predicción calculada con `fk.fk(q)`; poses candidatas
de `herramientas/verificar_fk.py`:

| Pose | q [rad] | x_pred | y_pred | z_pred |
|---|---|---:|---:|---:|
| `ready` | [0, −0.5, 0.5, 0, 0.5, 0] | 96.6 | −39.4 | 402.8 |
| `girada` | [0.6, −0.4, 0.4, 0, 0.3, 0] | 102.2 | 11.0 | 407.6 |
| `baja` | [0, −1.2, 1.2, 0, 0, 0] | 152.5 | −63.4 | 346.2 |
| `cero` (referencia) | [0, 0, 0, 0, 0, 0] | 50.0 | −63.4 | 416.3 |

Valores en mm. **Poses finales elegidas y congeladas:** `[ ]`, `[ ]`, `[ ]`
(commit de congelación: `[hash]`).

## 6. Resultados de la medición (rellenar después)

Los datos crudos van en `evidencias/medición_fk.csv`. Resumen:

| Pose | Error de posición [mm] | ¿≤ 10 mm? |
|---|---:|:-:|
| `[ ]` | `[ ]` | `[ ]` |
| `[ ]` | `[ ]` | `[ ]` |
| `[ ]` | `[ ]` | `[ ]` |

Análisis del error (si es grande y constante → offset de marco o de `d6`; si crece con la
distancia → revisar los `a_i`): `[ ]`

## 7. Firma

Firmado antes de ejecutar las mediciones:
`[Nombre 1]` · `[Nombre 2]` · `[Nombre 3]` · `[Nombre 4]` — Fecha/hora: `[ ]`

---

*Este documento se completa luego con: diagrama de secuencia (Ítem 2) y la predicción del p95
por política (Ítem 3), que forman parte del mismo «documento de diseño previo».*
