# Reto 2 — El Turno del Brazo (RB-2)

Cinemática directa y acceso al JetCobot  con **ROS 2 Humble**.

El objetivo principal es que cuatro clientes tienen un solo brazo. Y ningún cliente publica en `/joint_states`, solo el worker del
nodo `arm_broker` (en el Jetson) habla con el driver. El broker recibe goals por una acción,
los admite o rechaza con la FK, los encola según una política y los ejecuta de a uno.

```mermaid
 graph LR
    subgraph Clientes [Nodos Cliente - Raspberry Pi]
        C1(Cliente 1)

        C2(Cliente 2)

        C3(Cliente 3)

        C4(Cliente 4)
    end

    Broker[<b>arm_broker</b><br/>Jetson<br/><i>Admisión FK -> Cola -> Worker</i>]

    Robot((JetCobot))

    C1 & C2 & C3 & C4 -- "Goals (move_arm)" --> Broker
    Broker -- "/joint_states" --> Robot
    Broker -. "/arm/queue_state (5 Hz)" .-> Clientes
```

## Estado del proyecto

| Ítem | Contenido | Pts | Estado |
|---|---|:-:|---|
| 1 | Tabla DH + `fk(q)` + medición de 3 poses | 4 | `fk.py` y tabla DH (`docs/tabla_dh.*`) listos · predicciones de 3 poses calculadas · **falta medir en el robot** |
| 2 | Broker: cola, exclusión mutua, admisión con FK | 5 + 3 | `broker.py` y `politicas.py` implementados (admisión con rechazos con motivo, cupo atómico, worker único, cancelación, revalidación del paso) y probados con ROS simulado · **falta probarlo en el Jetson**, el diagrama de secuencia y el registro de rechazos real |
| 3 | Medición FIFO vs. Round Robin, bag + CSV + figura | 4 | Protocolo, predicción, script de corrida (`experimento_item3.sh`) y análisis (`metricas.py`) listos y probados con datos simulados · **faltan el ensayo con ROS 2 y las corridas oficiales** |
| 4 | Objetivo cartesiano (`send_coords`) auditado con la FK | 2 | `auditar_ik.py`, CSV de evidencia y documento listos y probados sin robot · **falta la sesión con el robot** |
| — | Diseño previo firmado y cierre reflexivo | 2 | Diseño previo (`docs/diseño_previo.*`) y cierre (`docs/cierre_reflexivo.*`) en borrador · **faltan firma, diagrama de secuencia y resultados** |

## Autoría

Implementación del equipo:

- `fk.py` (`DH`, `fk` y las validaciones), `broker.py` (`goal_callback`, `handle_accepted_callback`,
  `_worker`, `execute_callback` y lo que los rodea) y `politicas.py` (`FIFO`, `RoundRobin`);
- las ampliaciones de `cliente.py` (modo asíncrono, `inicio_unix`, validación de la traza) y de
  `analisis/metricas.py` (rechazos, violaciones de exclusión mutua, Jain a mitad de corrida, figura);
- las herramientas nuevas `auditar_ik.py`, `experimento_item3.sh` y `simular_politicas.py`, la traza
  provisional, las pruebas y los documentos de `docs/`.

## Estructura del repositorio

| Ruta | Contenido |
|---|---|
| `src/arm_broker_interfaces/` | Acción `MoveArm` y mensaje `QueueState` |
| `src/arm_broker/arm_broker/fk.py` | **Ítem 1**: tabla DH, `fk_matriz`, `fk`, límites, workspace, paso articular |
| `src/arm_broker/arm_broker/broker.py` | **Ítem 2**: nodo `arm_broker` (goal/accepted/execute callbacks, worker) |
| `src/arm_broker/arm_broker/politicas.py` | **Ítems 2–3**: políticas `FIFO` y `RoundRobin` |
| `src/arm_broker/arm_broker/cliente.py` | Cliente de carga (uno por integrante; modos secuencial y asíncrono) |
| `src/arm_broker/test/` | `test_esenciales.py` (ítems 1 y 2), `test_analisis.py` (ítem 3), `test_auditar_ik.py` (ítem 4) y `simulacion_ros.py` (dobles de ROS, sin pruebas) |
| `herramientas/verificar_fk.py` | Compara `fk(q)` con el robot (corre en el Jetson) |
| `herramientas/auditar_ik.py` | **Ítem 4**: pide un objetivo con `send_coords()` y audita el `q` real con la FK propia |
| `herramientas/experimento_item3.sh` | **Ítem 3**: corre una política con el protocolo fijo y deja la evidencia en `evidencias/item3/` |
| `herramientas/simular_politicas.py` | **Ítem 3**: modelo de cola para la predicción previa (usa las mismas clases de política) |
| `herramientas/generar_carga.py` | Genera trazas de poses reproducibles (misma semilla = mismo CSV) |
| `trazas/prueba.csv` | Traza **provisional** para desarrollo (no es la oficial del docente) |
| `analisis/exportar_csv.py` | Bag → `queue_state.csv` y `joint_states.csv` |
| `analisis/metricas.py` | Espera media/p95/máxima, inanición, Jain, rechazos con causas, violaciones de exclusión mutua y figura comparativa |
| `docs/tabla_dh.md` / `.pdf` | Tabla DH, convención y predicciones de las 3 poses |
| `docs/diseño_previo.md` / `.pdf` | Diseño previo: predicción, protocolo y comandos del ítem 3 |
| `docs/ensayo_previo_ros2.md` | Paso a paso del ensayo con ROS 2 antes de las corridas oficiales |
| `docs/item4_auditoria_ik.md` | Qué demuestra el ítem 4, tabla de evidencia y lista de verificación |
| `docs/cierre_reflexivo.md` / `.pdf` | Borrador del cierre reflexivo |
| `evidencias/` | `medición_fk.csv` (ítem 1), `item3/` (una carpeta por política, la figura y los resultados) e `item4/auditoria_ik.csv` |

## Reglas del reto y cómo se cubren

- **Publicador único:** solo el worker de `arm_broker` publica en `/joint_states`
  y `cliente.py` solo envía goals a `move_arm`.
- **Encolar no es igual a ejecutar:** `handle_accepted_callback` solo encola, luego un solo único hilo worker
  desencola y ejecuta, y espera a `pedido.fin` antes de sacar el siguiente.
- **Callback groups:** `ReentrantCallbackGroup` para aceptar goals mientras otro se ejecuta, tambien
  `MutuallyExclusiveCallbackGroup` para el worker del brazo (sera un timer y no un hilo suelto), pues el
  timer de `/arm/queue_state` va en un grupo aparte.
- **La tarea de FK:** `goal_callback` rechaza con motivo legible si el objetivo está fuera
  de límites articulares, fuera del workspace o con paso articular excesivo.
- **Dominio DDS propio:** usamos `ROS_DOMAIN_ID` propio.

## Requisitos

- ROS 2 Humble (Ubuntu 22.04) en el Jetson y en las máquinas cliente.
- `rmw_fastrtps_cpp`.
- `pymycobot` en el Jetson (`pip install pymycobot`) para `verificar_fk.py` y `auditar_ik.py` (para Ítem 4).
- `matplotlib` para la figura de `metricas.py` (`pip install matplotlib`).
- `rosbag2` (`ros-humble-rosbag2` y `ros-humble-rosbag2-storage-default-plugins`) para grabar y exportar el bag del Ítem 3.

## Ítem 1 — Cinemática directa

1. La tabla DH y su justificación están en [`docs/tabla_dh.md`](docs/tabla_dh.md) (PDF en
   `docs/tabla_dh.pdf`). La implementación está en `fk.py`; el código y la tabla deben coincidir.
2. Antes de medir, se congelan las predicciones de 3 poses en un commit (`fk.fk(q)`), tal
   como se documenta en la sección 5 del documento de diseño.
3. Llevar el brazo a cada pose y medir físicamente con una regla la posición del efector
   respecto al origen del marco {0}. Registrar los resultados en `evidencias/medición_fk.csv`.
4. Chequeo cruzado con el firmware, en el Jetson y con el puerto de serie libre:

   ```bash
   python3 herramientas/verificar_fk.py              # mueve el brazo por las poses de prueba
   python3 herramientas/verificar_fk.py --solo-leer  # solo compara donde está el brazo
   ```

   Nota: Esto compara contra `get_coords()` del firmware; no reemplaza la medición física

5. Criterio de aceptación: Error de posición <= 10 mm en las tres poses.

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

**Pruebas esenciales** (una por fila de la tabla de pruebas esenciales), en `test_esenciales.py`:

```bash
cd src/arm_broker && python3 -m unittest discover -s test -p "test_esenciales.py" -v
```

| Ítem | Prueba esencial | Qué demuestra |
|---|---|---|
| 1 FK | pose cero, predicciones de las 3 poses del diseño previo, cálculo del error | La FK y el criterio de error ≤ 10 mm (las 3 poses físicas necesitan el robot) |
| 2 | Goal válido → `ACCEPT` | El broker acepta una solicitud correcta |
| 2 | Límite articular → `REJECT` | Rechazo con motivo |
| 2 | Workspace → `REJECT` | Rechazo con motivo |
| 2 | Paso excesivo → `REJECT` | Rechazo con motivo |
| 2 | `handle_accepted` solo encola | Encolar ≠ ejecutar |
| 2 | Máximo 1 goal ejecutándose | Exclusión mutua |
| 2 | Solo el broker publica `/joint_states` | Regla de oro |
| 2 | FIFO y Round Robin generan el orden esperado | Las políticas funcionan |

Para los ítems 3 y 4 solo se conserva lo que se puede probar sin robot (sus corridas y la prueba
física necesitan el robot):

| Ítem | Archivo | Prueba | Qué demuestra |
|---|---|---|---|
| 3 | `test_analisis.py` | métricas completas sobre CSV sintéticos (y la figura, si hay matplotlib) | `metricas.py` reporta todo lo pedido |
| 3 | `test_analisis.py` | corrida limpia: 0 violaciones; goals intercalados: se detectan | Exclusión mutua = 0 |
| 3 | `test_analisis.py` | órdenes de FIFO y Round Robin con la misma carga | Las dos políticas con las mismas condiciones |
| 4 | `test_auditar_ik.py` | `error_cartesiano((0,0,0),(3,4,0)) == 5.0` | La fórmula del error |
| 4 | `test_auditar_ik.py` | grados → radianes | La conversión antes de la FK |
| 4 | `test_auditar_ik.py` | flujo con un brazo simulado: `send_coords` y auditoría del `q` leído | El flujo del ítem 4 |
| 4 | `test_auditar_ik.py` | sin objetivo o sin confirmación, no se mueve el brazo | Seguridad |

Todas se corren con:

```bash
cd src/arm_broker && python3 -m unittest discover -s test -v
```

Para `simulacion_ros.py` no es necesario ROS 2, pues sustituye `rclpy` por dobles,
incluida la máquina de estados de los goals. Las pruebas del broker se omiten si ROS 2 está
instalado y no sustituyen la prueba real: falta correr el broker y los clientes en el Jetson.

Se retiraron las pruebas ampliadas de los ítems 2, 3 y 4 (cancelación, excepciones, cupo
concurrente, UUID, validación del CSV y del modo asíncrono, revalidación del paso, métricas
auxiliares, registro CSV de la auditoría…); el comportamiento sigue en el código y las pruebas están
en el historial de git (por ejemplo `git show 012d0de:src/arm_broker/test/test_broker_simulado.py`).

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

Un único objetivo cartesiano `(x, y, z)` sobre la mesa, resuelto por el **firmware** con
`send_coords()` (pymycobot): **el equipo no escribe un solver de IK**. Luego se lee el `q`
realmente ejecutado (`get_angles()`, en grados), se pasa a radianes, se calcula `FK(q_real)` con
`fk.py` y se mide el error contra lo solicitado (≤ 10 mm). `get_coords()` es dato de apoyo.

```
objetivo → send_coords() → el firmware resuelve la IK → el brazo adopta q
        → get_angles() [°] → q [rad] → FK(q_real) → e = |FK(q_real) − objetivo|
```

```bash
python3 herramientas/auditar_ik.py --plantilla          # tabla vacía, sin robot
python3 herramientas/auditar_ik.py --solo-leer          # no mueve; diagnóstico FK vs get_coords()
python3 herramientas/auditar_ik.py --x X --y Y --z Z --rx RX --ry RY --rz RZ \
    --confirmo-espacio-despejado --guardar              # la auditoría; agrega a evidencias/item4/
```

No hay objetivo por defecto: el punto seguro sobre el tablero se mide y se define en la sesión. La
evidencia queda en `evidencias/item4/auditoria_ik.csv`. Detalle, tabla de evidencia, cómo leer el
resultado y la pregunta de la semana 5 (¿por qué esa solución y no la del codo contrario?) en
[`docs/item4_auditoria_ik.md`](docs/item4_auditoria_ik.md).

- [ ] Objetivo pedido, `q` ejecutado, `FK(q)` y error: `[completar]`

## Lista de entregables

- [x] Paquetes `arm_broker` y `arm_broker_interfaces` en GitHub.
- [x] README con instrucciones de ejecución (este archivo; quedan los campos `[completar]` del equipo).
- [ ] Documento de diseño previo **firmado antes de medir**: tabla DH, diagrama de secuencia y
      predicción del p95 por política (`docs/`; borrador listo, faltan firma y diagrama de secuencia).
- [ ] Bag, CSV y figura comparativa de las políticas (`evidencias/item3/`).
- [ ] Medición de las 3 poses del ítem 1 (`evidencias/medición_fk.csv`) y auditoría del ítem 4
      (`evidencias/item4/auditoria_ik.csv`).
- [ ] Video de 3 minutos con los cuatro clientes en disputa y `/arm/queue_state` en pantalla.
- [ ] Cierre reflexivo (máximo una página): ¿qué política llevarían a CapyTown y por qué? (`docs/cierre_reflexivo.*`; borrador listo).

## Rúbrica (20 pts)

| Criterio | Pts | Dónde se evidencia |
|---|:-:|---|
| FK correcta y verificada contra el robot real | 4 | `fk.py`, `docs/tabla_dh.*`, `evidencias/medición_fk.csv` |
| Broker: exclusión mutua y encolado correcto | 5 | `broker.py`, `politicas.py`, bag de `/joint_states` |
| Admisión validada con FK y rechazos razonados | 3 | `goal_callback`, registro de rechazos |
| Medición y comparación de políticas | 4 | `evidencias/`, figura comparativa |
| Ítem 4: error cartesiano auditado con FK propia | 2 | `auditar_ik.py`, `evidencias/item4/`, `docs/item4_auditoria_ik.md` |
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
