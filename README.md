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
| 2 | Broker: cola, exclusión mutua, admisión con FK | 5 + 3 | **Implementado** (`broker.py`, `politicas.py`) y probado con ROS simulado · **falta probar en el Jetson**, diagrama de secuencia y registro de rechazos real |
| 3 | Medición FIFO vs. política elegida, bag + CSV + figura | 4 | Protocolo, predicción, script de corrida y análisis listos y probados con datos simulados · **faltan el ensayo con ROS 2 y las corridas oficiales** |
| 4 | Objetivo cartesiano (`send_coords`) auditado con la FK | 2 | Pendiente |
| — | Diseño previo firmado y cierre reflexivo | 2 | Diseño previo en borrador (`docs/`) · cierre pendiente |

## Autoría

La estructura del proyecto (manifiestos, `CMakeLists.txt`, `setup.py`, interfaces, publicador de
`/arm/queue_state`, `cliente.py` y los scripts de `analisis/` y `herramientas/`) la entrega el
curso y es idéntica para todos los equipos. El trabajo propio del equipo está en los bloques
`IMPLEMENTAR`: `fk.py` (`DH`, `fk`), `broker.py` (`goal_callback`, `handle_accepted_callback`,
`_worker`, `execute_callback`) y `politicas.py` (`FIFO`, `RoundRobin`).

## Estructura del repositorio

| Ruta | Contenido |
|---|---|
| `src/arm_broker_interfaces/` | Acción `MoveArm` y mensaje `QueueState` |
| `src/arm_broker/arm_broker/fk.py` | **Ítem 1**: tabla DH, `fk_matriz`, `fk`, límites, workspace, paso articular |
| `src/arm_broker/arm_broker/broker.py` | **Ítem 2**: nodo `arm_broker` (goal/accepted/execute callbacks, worker) |
| `src/arm_broker/arm_broker/politicas.py` | **Ítems 2–3**: políticas `FIFO` y `RoundRobin` |
| `src/arm_broker/test/test_politicas.py` | Pruebas unitarias de las políticas (no requieren ROS) |
| `src/arm_broker/arm_broker/cliente.py` | Cliente de carga (uno por integrante) |
| `herramientas/verificar_fk.py` | Compara `fk(q)` con el robot (corre en el Jetson) |
| `herramientas/generar_carga.py` | Genera trazas de poses reproducibles (misma semilla = mismo CSV) |
| `analisis/exportar_csv.py` | Bag → `queue_state.csv` y `joint_states.csv` |
| `analisis/metricas.py` | Espera media/p95/máxima, inanición, Jain, rechazos con causas, violaciones de exclusión mutua y figura comparativa |
| `herramientas/experimento_item3.sh` | Corre una política con el protocolo fijo y deja la evidencia en `evidencias/item3/` |
| `herramientas/simular_politicas.py` | Modelo de cola para la predicción previa (usa las mismas clases de política) |
| `trazas/prueba.csv` | Traza **provisional** para desarrollo (no es la oficial) |
| `docs/tabla_dh.md` / `.pdf` | Documento de diseño previo: tabla DH y predicciones |
| `docs/diseño_previo.md` / `.pdf` | Diseño previo: predicción, protocolo y comandos del ítem 3 |
| `docs/ensayo_previo_ros2.md` | Paso a paso del ensayo con ROS 2 antes de las corridas oficiales |
| `docs/cierre_reflexivo.md` / `.pdf` | Borrador del cierre reflexivo |
| `evidencias/` | `medición_fk.csv` (ítem 1) e `item3/` (una carpeta por política, la figura y los resultados) |

## Reglas del reto y cómo las cubre el diseño


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
ros2 run arm_broker broker --ros-args -p politica:=round_robin
```

El driver `sync_plan_nx` debe estar corriendo en el Jetson (una sola persona lo levanta).

Parámetros del broker: `politica` (`fifo` | `round_robin`), `cola_max` (20), `paso_max_rad`
(1.2), `duracion_movimiento_s` (3.0), `pasos_interpolacion` (10) y `archivo_rechazos`
(`rechazos.csv`; vacío para desactivar el registro).

**En cada máquina cliente** (uno por integrante, cada cual con su `client_id` y prioridad):

```bash
ros2 run arm_broker cliente --ros-args \
  -r __node:=cliente_1 \
  -p client_id:=ana -p priority:=3 -p traza:=/ruta/traza_oficial.csv -p repeticiones:=1
```

Para generar cola de verdad (varios pedidos pendientes a la vez) se usa el modo asíncrono, que
envía toda la traza sin esperar a que cada goal termine y recoge los resultados al final:

```bash
ros2 run arm_broker cliente --ros-args -r __node:=cliente_1 \
  -p client_id:=A -p priority:=1 -p traza:=/ruta/traza_oficial.csv \
  -p modo:=asincrono -p pausa_s:=0.0
```

`modo` es `secuencial` (por defecto: un goal a la vez, sirve para probar un movimiento) o
`asincrono`. Con `pausa_s` en 0 los goals de cada cliente llegan seguidos y compiten entre sí.
Con cuatro clientes asíncronos a la vez, FIFO y Round Robin atienden en órdenes distintos; en el
modo secuencial cada cliente tiene un solo pedido en cola y las dos políticas se parecen mucho.

`inicio_unix` (segundos Unix, por defecto 0 = enviar ya) hace que el cliente espere a un instante
común antes de enviar el primer goal; así varios clientes arrancan igual aunque cada `ros2 run`
tarde distinto en levantar.

La traza se valida al cargarla: cada fila no vacía (ni comentario `#`) debe tener exactamente
6 números finitos; si no, el cliente termina con un error que indica el archivo y la línea.

`-r __node:=cliente_N` le da a cada cliente un nombre de nodo distinto (`cliente_1`, `cliente_2`,
…); sin él todos se llaman `arm_client` y no se distinguen en `ros2 node list`. `client_id` es
el nombre que aparece en `/arm/queue_state`.

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

Además se rechaza con causa `cola_llena` si hay `cola_max` pedidos pendientes. Cada rechazo de
`goal_callback` se muestra en el log del broker y se agrega a `rechazos.csv` (`t_unix, client_id,
priority, causa, motivo, joint_positions`), que es la evidencia del registro de rechazos. La causa
es una de `limite`, `workspace`, `paso` o `cola_llena`.

Contadores de `/arm/queue_state`: `total_accepted` es el acumulado de goals que pasaron la
admisión, `total_rejected` cuenta solo los rechazados por `goal_callback` (los mismos que están
en `rechazos.csv`) y `total_completed` los que terminaron con éxito.

Cómo está armado el broker (`broker.py`):

- `goal_callback`: solo calcula con `fk.py` y devuelve ACCEPT/REJECT; nunca espera al brazo.
  El paso articular se mide contra `q_actual` en el momento de la admisión. El cupo de la cola
  se comprueba y se **reserva en una sola operación bajo el lock** (`self.reservados`), para que
  goals simultáneos no superen `cola_max`; la reserva se convierte en pedido real en
  `handle_accepted_callback`.
- `handle_accepted_callback`: crea el `Pedido` y lo añade a `pendientes`. No ejecuta ni publica.
- `_worker`: es un callback periódico (timer de 20 ms) del `MutuallyExclusiveCallbackGroup`, por
  lo que dos vueltas nunca se solapan. Cada vuelta elige **un** pedido con `politica.siguiente()`,
  llama a `goal_handle.execute()` y espera `pedido.fin` antes de volver; esa espera es la
  exclusión mutua. Los pedidos cancelados mientras esperaban se descartan sin mover el brazo.
  El timer de `/arm/queue_state` tiene su propio grupo, para seguir publicando mientras el worker
  espera, y el executor usa 4 hilos porque el worker ocupa uno mientras `execute_callback` usa otro.
- `execute_callback`: **vuelve a validar el paso articular** contra `q_actual` justo antes de
  mover, porque los pedidos de delante pudieron cambiar la pose desde la admisión. Si ya no es
  seguro, el goal termina `aborted` con `success=False` y el motivo en `message`, no mueve el
  brazo y queda en el log como `ABORTADO: paso_al_ejecutar`. No cambia `total_accepted` ni
  `total_rejected` ni entra en `rechazos.csv`, que son solo de `goal_callback`.
  Si es seguro, interpola desde `q_actual` hasta el destino en `pasos_interpolacion`
  pasos, publica feedback `EXECUTING` en cada uno, revisa la cancelación entre pasos y devuelve
  `wait_time_s` y `exec_time_s`. Al final siempre libera `pedido.fin`.
- Cancelación: `rclpy` solo envía el `Result` al cliente desde `execute_callback`, así que un
  pedido cancelado en la cola también pasa por `goal_handle.execute()` (que en ese caso no lo
  pasa a EXECUTING) y `execute_callback` lo cierra como `canceled` sin mover el brazo, con
  `success=False`, `message='cancelado mientras esperaba en cola'`, `wait_time_s` = lo que
  esperó y `exec_time_s=0.0`. Si se cancela durante la ejecución, el brazo se queda en la
  última pose publicada (no vuelve) y el mensaje es `cancelado durante la ejecución…`.
- Errores internos: si `_atender` falla, se registra el error y el pedido se da por fallido
  (`aborted`, `success=False`, `pedido.resultado` asignado y `pedido.fin` liberado), de modo que
  ni el worker ni el cliente quedan bloqueados.
- Los `goal_id` son la UUID completa (32 caracteres hexadecimales), sin recortar.

`/arm/queue_state` (`arm_broker_interfaces/msg/QueueState`) se publica a 5 Hz con el cliente en
ejecución, la longitud de la cola, las esperas y los totales aceptados/rechazados/completados.

- [ ] Diagrama de secuencia (`docs/`).
- [ ] Registro de rechazos con motivo (evidencia).

### Pruebas

```bash
cd src/arm_broker && python3 -m unittest discover -s test -v
```

No necesitan ROS 2 ni el robot. `test_broker_simulado.py` sustituye `rclpy` por dobles de
prueba —incluida la máquina de estados de los goals, de modo que un `abort()` o `canceled()`
inválido falla igual que en ROS— y verifica:

| Prueba | Qué comprueba |
|---|---|
| `TestAdmision` | goal válido aceptado; límites, workspace y paso rechazados con motivo (también en `rechazos.csv`); cola llena; cupo exacto con 40 goals simultáneos; UUID completa |
| `TestEjecucion` | nunca más de un `execute_callback` a la vez y ningún mensaje en `/joint_states` sin un goal ejecutando; tiempos y feedback; revalidación del paso al ejecutar |
| `TestCancelacion` | cancelación en cola (el cliente recibe su `Result`) y en ejecución; una excepción en `_atender` no bloquea al cliente ni al worker |
| `TestPoliticasEnElBroker` | con la cola `A1 A2 A3 B1 B2 C1 C2` pendiente, FIFO atiende `A1 A2 A3 B1 B2 C1 C2` y Round Robin `A1 B1 C1 A2 B2 C2 A3`, también con tres `Cliente` reales en modo asíncrono |
| `TestCliente` | validación estricta de la traza CSV y del parámetro `modo` |
| `test_analisis.py` | `metricas.py` (violaciones, Jain a mitad, rechazos, figura) y `simular_politicas.py` (órdenes de FIFO y Round Robin) con CSV sintéticos |
| `TestUnicoPublicador` | análisis estático: solo `broker.py` crea publicadores, solo `mover()` publica en `/joint_states` y `cliente.py` no publica nada |

Las pruebas simuladas se omiten si ROS 2 está instalado. Sus resultados no sustituyen la
prueba real: falta correr el broker y los clientes en el Jetson con el robot.

## Ítem 3 — Medición bajo contención

Políticas comparadas:

- **FIFO** (obligatoria).
- **Round Robin entre clientes** (`round_robin`), la política elegida. Recorre los clientes en
  orden circular a partir del último atendido y toma el más antiguo del primero que tenga algo
  pendiente. **Ignora la prioridad numérica.**
  - *Por qué:* con clientes que mandan ráfagas, FIFO hace esperar mucho más al que llega después;
    Round Robin reparte el turno y iguala las esperas medias por cliente.
  - *Lo que no hace:* no cambia la espera media global ni acorta las esperas extremas (el último
    goal de cada cliente sigue esperando casi toda la corrida), y según el orden de llegada puede
    empeorar el índice de inanición. Está predicho en `docs/diseño_previo.md` y se discute en el
    cierre reflexivo.

Convención de prioridad: **mayor número = más urgente** (ver `MoveArm.action`).

### Experimento justo

Las dos corridas usan las mismas condiciones y cambia solo `politica:=`: misma traza oficial del
docente, cuatro clientes con las mismas prioridades (A=1, B=2, C=3, D=4), mismas repeticiones,
`modo:=asincrono`, misma `pausa_s`, misma duración e interpolación y el mismo procedimiento de
inicio (broker → `ros2 bag record` → clientes con un `inicio_unix` común y 0.5 s de escalón).
El protocolo completo, la predicción del p95 y la plantilla de resultados están en
[`docs/diseño_previo.md`](docs/diseño_previo.md).

```bash
# Una corrida por política; deja todo en evidencias/item3/<política>/ (falla si ya hay resultados)
TRAZA=/ruta/traza_oficial.csv bash herramientas/experimento_item3.sh fifo
TRAZA=/ruta/traza_oficial.csv bash herramientas/experimento_item3.sh round_robin

# Métricas y figura comparativa
python3 analisis/metricas.py evidencias/item3/fifo/queue_state.csv \
    evidencias/item3/round_robin/queue_state.csv \
    --salida evidencias/item3/comparacion_politicas.png
```

```
evidencias/item3/
├── fifo/         bag/, queue_state.csv, joint_states.csv, rechazos.csv, protocolo.txt, logs
├── round_robin/  (igual)
├── comparacion_politicas.png
└── resultados.md (copia de resultados_plantilla.md, completada después de medir)
```

Antes de las corridas oficiales se hace el ensayo con la traza provisional
([`docs/ensayo_previo_ros2.md`](docs/ensayo_previo_ros2.md)): broker a ~5 Hz en
`/arm/queue_state`, un solo publicador en `/joint_states`, cola con varios goals, `ros2 bag`,
exportación y análisis.

| Métrica | Fuente | Criterio |
|---|---|---|
| Violaciones de exclusión mutua | `metricas.py`: goals intercalados y mensajes de `/joint_states` sin goal ejecutando; más `ros2 topic info /joint_states -v` | **Cero** (eliminatorio) |
| Espera media y p95 por prioridad | `/arm/queue_state` → CSV | Se compara entre políticas |
| Índice de inanición | Espera máxima de la prioridad más baja | Se discute en el cierre |
| Equidad de Jain | Goals atendidos por cliente (al final y a mitad de la corrida) | Se reporta el valor |
| Goals rechazados | `total_rejected` y `rechazos.csv` del broker | Cantidad y causas |
| Error de FK | Medición física (Ítem 1) y FK(q) (Ítem 4) | ≤ 10 mm |

Notas sobre la medición:

- Al final de una traza finita todos los clientes fueron atendidos por igual y Jain vale 1.000 en
  ambas políticas; por eso `metricas.py` también reporta Jain **a mitad de la corrida**.
- Las esperas tienen resolución de ~0.2 s (`/arm/queue_state` a 5 Hz) y no incluyen los goals que
  empiezan a ejecutarse antes de la siguiente publicación.
- La detección de violaciones desde los CSV es un indicio (un bag no dice quién publicó cada
  mensaje); la prueba de publicador único es `ros2 topic info /joint_states -v`, que el script
  guarda en `publicadores_joint_states.txt`.

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
