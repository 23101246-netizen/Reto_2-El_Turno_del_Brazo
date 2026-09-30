# Reto 2 — El Turno del Brazo (RB-2)

Cinemática directa y acceso concurrente al JetCobot  con **ROS 2 Humble**.

Cuatro clientes, un solo brazo. Ningún cliente publica en `/joint_states`: solo el worker del
nodo `arm_broker` (en el Jetson) habla con el driver. El broker recibe goals por una acción,
los admite o rechaza con la FK, los encola según una política y los ejecuta de a uno.

```
 cliente 1 ─┐
 cliente 2 ─┤  goals (acción move_arm)      ┌────────────────────────────┐
 cliente 3 ─┼──────────────────────────────▶│ arm_broker (Jetson)        │──▶ /joint_states ──▶ JetCobot
 cliente 4 ─┘                               │ admisión FK → cola → worker│
        ▲                                   └─────────────┬──────────────┘
        └──────────────── /arm/queue_state (5 Hz) ◀───────┘
```

## Estado del proyecto

Actualizar esta tabla a medida que se avanza.

| Ítem | Contenido | Pts | Estado |
|---|---|:-:|---|
| 1 | Tabla DH + `fk(q)` + medición de 3 poses | 4 | Código listo · tabla DH en borrador · **falta medir en el robot** |
| 2 | Broker: cola, exclusión mutua, admisión con FK | 5 + 3 | Andamiaje entregado · **falta implementar** (`broker.py`, `politicas.py`) |
| 3 | Medición FIFO vs. política elegida, bag + CSV + figura | 4 | Herramientas de análisis listas · **faltan corridas** |
| 4 | Objetivo cartesiano (`send_coords`) auditado con la FK | 2 | Pendiente |
| — | Diseño previo firmado y cierre reflexivo | 2 | Diseño previo en borrador (`docs/`) · cierre pendiente |

## Estructura del repositorio

| Ruta | Contenido |
|---|---|
| `src/arm_broker_interfaces/` | Acción `MoveArm` y mensaje `QueueState` |
| `src/arm_broker/arm_broker/fk.py` | **Ítem 1**: tabla DH, `fk_matriz`, `fk`, límites, workspace, paso articular |
| `src/arm_broker/arm_broker/broker.py` | **Ítem 2**: nodo `arm_broker` (goal/accepted/execute callbacks, worker) |
| `src/arm_broker/arm_broker/politicas.py` | **Ítems 2–3**: `FIFO` y la segunda política |
| `src/arm_broker/arm_broker/cliente.py` | Cliente de carga (uno por integrante) |
| `herramientas/verificar_fk.py` | Compara `fk(q)` con el robot (corre en el Jetson) |
| `herramientas/generar_carga.py` | Genera trazas de poses reproducibles (misma semilla = mismo CSV) |
| `analisis/exportar_csv.py` | Bag → `queue_state.csv` y `joint_states.csv` |
| `analisis/metricas.py` | Espera media/p95, inanición, Jain y figura comparativa |
| `docs/tabla_dh.md` / `.pdf` | Documento de diseño previo: tabla DH y predicciones |
| `evidencias/` | `medición_fk.csv`, y un subdirectorio por política con sus CSV |

## Reglas del reto y cómo las cubre el diseño

(El andamiaje ya las fija; se cumplen del todo cuando se implementen `broker.py` y `politicas.py`.)

- **Publicador único:** solo el worker de `arm_broker` publica en `/joint_states`
  (`ArmBroker.mover`). `cliente.py` solo envía goals a `move_arm`.
- **Encolar ≠ ejecutar:** `handle_accepted_callback` solo encola; un único hilo worker
  desencola y ejecuta, y espera `pedido.fin` antes de sacar el siguiente.
- **Callback groups:** `ReentrantCallbackGroup` para aceptar goals mientras otro se ejecuta;
  `MutuallyExclusiveCallbackGroup` para el worker del brazo.
- **La FK es el portero:** `goal_callback` rechaza con motivo legible si el objetivo está fuera
  de límites articulares, fuera del workspace o con paso articular excesivo.
- **Dominio DDS propio:** cada equipo usa su `ROS_DOMAIN_ID`.

## Requisitos

- ROS 2 Humble (Ubuntu 22.04) en el Jetson y en las máquinas cliente.
- `rmw_fastrtps_cpp`.
- `pymycobot` en el Jetson (`pip install pymycobot`) para `verificar_fk.py` y el Ítem 4.
- `matplotlib` para la figura de `metricas.py` (`pip install matplotlib`).

## Configuración de red

Cada integrante exporta estas variables en cada terminal (ver el laboratorio previo para el
archivo `super_client_configuration_file.xml`):

```bash
export ROS_DOMAIN_ID=<42 + n.º de equipo>        # asignado por el docente
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<IP del Jetson del equipo>:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=~/super_client_configuration_file.xml
ros2 daemon stop && ros2 daemon start
```

Si `ros2 node list` sale vacío, el problema es de descubrimiento, no del robot.

## Compilar

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select arm_broker_interfaces arm_broker
source install/setup.bash
```

## Ejecución

**En el Jetson**: lanzar el broker con la política a medir.

```bash
ros2 run arm_broker broker --ros-args -p politica:=fifo
# la segunda política:
ros2 run arm_broker broker --ros-args -p politica:=prioridad -p tau_envejecimiento_s:=8.0
```

Parámetros del broker: `politica`, `tau_envejecimiento_s`, `cola_max` (20), `paso_max_rad`
(1.2), `duracion_movimiento_s` (3.0), `pasos_interpolacion` (10).

**En cada máquina cliente** (uno por integrante, cada cual con su `client_id` y prioridad):

```bash
ros2 run arm_broker cliente --ros-args \
  -p client_id:=ana -p priority:=3 -p traza:=/ruta/traza_oficial.csv -p repeticiones:=1
```

La traza es el CSV oficial del docente (mismo archivo para todos los equipos). Para ensayos
locales se puede generar una: `python3 herramientas/generar_carga.py --n 40 --semilla 7 --salida carga.csv`.

**Observar la cola** (visible para todos los clientes, a 5 Hz):

```bash
ros2 topic echo /arm/queue_state
```

## Ítem 1 — Cinemática directa

1. La tabla DH y su justificación están en [`docs/tabla_dh.md`](docs/tabla_dh.md) (PDF en
   `docs/tabla_dh.pdf`). La implementación está en `fk.py`; el código y la tabla deben coincidir.
2. **Antes de medir**, se congelan las predicciones de 3 poses en un commit (`fk.fk(q)`), tal
   como se documenta en la sección 5 del documento de diseño.
3. Llevar el brazo a cada pose y **medir físicamente** con regla o cinta la posición del efector
   respecto al origen del marco {0}. Registrar los resultados en `evidencias/medición_fk.csv`.
4. Chequeo cruzado con el firmware, en el Jetson y con el puerto serie libre:

   ```bash
   python3 herramientas/verificar_fk.py              # mueve el brazo por las poses de prueba
   python3 herramientas/verificar_fk.py --solo-leer  # solo compara donde está el brazo
   ```

   Nota: esto compara contra `get_coords()` del firmware; **no reemplaza la medición física**
   que exige el reto.
5. **Criterio de aceptación:** error de posición ≤ 10 mm en las tres poses.

## Ítem 2 — Broker

Flujo de un goal: **admisión** (`goal_callback`, barata e inmediata) → **encolado**
(`handle_accepted_callback`) → **ejecución exclusiva** (worker único → `execute_callback`).

Rechazos con motivo, todos calculados con `fk.py`:

| Causa | Función | Ejemplo de motivo |
|---|---|---|
| Límite articular | `fk.dentro_de_limites` | `2_Joint fuera de rango: 2.500 rad, límite [-2.36, 2.36]` |
| Workspace | `fk.dentro_del_workspace` | `efector a 512 mm de la base, máximo 480` |
| Paso excesivo | `fk.paso_articular` | paso mayor que `paso_max_rad` |

`/arm/queue_state` (`arm_broker_interfaces/msg/QueueState`) se publica a 5 Hz con el cliente en
ejecución, la longitud de la cola, las esperas y los totales aceptados/rechazados/completados.

- [ ] Diagrama de secuencia (`docs/`).
- [ ] Registro de rechazos con motivo (evidencia).

## Ítem 3 — Medición bajo contención

Políticas comparadas:

- **FIFO** (obligatoria).
- **Segunda política:** `[completar: prioridad estática / prioridad con envejecimiento / round-robin]`
  — justificación: `[completar]`. Los directorios de `evidencias/` deben llevar el nombre de la
  política medida (hoy: `fifo/` y `round_robin/`); si la segunda es otra, renombrar.

Convención de prioridad: **mayor número = más urgente** (ver `MoveArm.action`).

Procedimiento por política (mismo CSV de traza en todas las corridas):

```bash
# 1. lanzar el broker con la política, y los 4 clientes a la vez
# 2. grabar
ros2 bag record -o bag_fifo /joint_states /arm/queue_state
# 3. exportar
python3 analisis/exportar_csv.py bag_fifo --salida evidencias/fifo
# 4. métricas y figura comparativa
python3 analisis/metricas.py evidencias/fifo/queue_state.csv evidencias/round_robin/queue_state.csv \
        --salida comparacion_politicas.png
```

| Métrica | Fuente | Criterio |
|---|---|---|
| Violaciones de exclusión mutua | Bag de `/joint_states` (publicadores concurrentes) | **Cero** (eliminatorio) |
| Espera media y p95 por prioridad | `/arm/queue_state` → CSV | Se compara entre políticas |
| Índice de inanición | Espera máxima de la prioridad más baja | Se discute en el cierre |
| Equidad de Jain | Goals atendidos por cliente | Se reporta el valor |
| Goals rechazados | Registro de `goal_callback` | Todo rechazo lleva motivo |
| Error de FK | Medición física (Ítem 1) y FK(q) (Ítem 4) | ≤ 10 mm |

Para verificar cero violaciones: `ros2 topic info /joint_states --verbose` debe mostrar un único
publicador (el nodo `arm_broker`).

## Ítem 4 — Puerta a la cinemática inversa

Un único objetivo cartesiano `(x, y, z)` sobre la mesa, resuelto por el firmware con
`send_coords()` (pymycobot). Luego se lee el `q` realmente ejecutado, se calcula `FK(q)` con
`fk.py` y se mide el error contra lo solicitado (≤ 10 mm). Pregunta abierta para la semana 5:
¿por qué el brazo eligió esa solución y no la del codo contrario?

- [ ] Objetivo pedido, `q` ejecutado, `FK(q)` y error: `[completar]`

## Lista de entregables

- [x] Paquetes `arm_broker` y `arm_broker_interfaces` en GitHub.
- [ ] README con instrucciones de ejecución (este archivo; completar los campos `[completar]`).
- [ ] Documento de diseño previo **firmado antes de medir**: tabla DH, diagrama de secuencia y
      predicción del p95 por política (`docs/`).
- [ ] Bag, CSV y figura comparativa de las políticas (`evidencias/`).
- [ ] Video de 3 minutos con los cuatro clientes en disputa y `/arm/queue_state` en pantalla.
- [ ] Cierre reflexivo (máximo una página): ¿qué política llevarían a CapyTown y por qué?

## Rúbrica (20 pts)

| Criterio | Pts | Dónde se evidencia |
|---|:-:|---|
| FK correcta y verificada contra el robot real | 4 | `fk.py`, `docs/tabla_dh.*`, `evidencias/medición_fk.csv` |
| Broker: exclusión mutua y encolado correcto | 5 | `broker.py`, `politicas.py`, bag de `/joint_states` |
| Admisión validada con FK y rechazos razonados | 3 | `goal_callback`, registro de rechazos |
| Medición y comparación de políticas | 4 | `evidencias/`, figura comparativa |
| Ítem 4: error cartesiano auditado con FK propia | 2 | sección Ítem 4 |
| Diseño previo y cierre reflexivo | 2 | `docs/`, cierre reflexivo |

**Penalización:** cualquier cliente que publique directamente en `/joint_states` anula el
puntaje del criterio de exclusión mutua.

## Equipo

| Integrante | Rol / `client_id` | Prioridad |
|---|---|:-:|
| `[completar]` | | |
| `[completar]` | | |
| `[completar]` | | |
| `[completar]` | | |

Equipo n.º `[completar]` · `ROS_DOMAIN_ID` = `[completar]`
