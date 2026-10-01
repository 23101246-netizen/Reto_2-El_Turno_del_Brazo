# Ensayo previo con ROS 2 (traza provisional)

Sirve para validar, **antes** de las corridas oficiales, todo el flujo: broker → clientes → bag → CSV →
métricas. Usa `trazas/prueba.csv` (provisional, solo desarrollo). No necesita el robot: el broker
publica `/joint_states` aunque nadie lo escuche.

```bash
source /opt/ros/humble/setup.bash
cd <workspace> && colcon build --packages-select arm_broker_interfaces arm_broker
source install/setup.bash
export ROS_DOMAIN_ID=<42 + n.º de equipo>
```

## 1. Broker

```bash
ros2 run arm_broker broker --ros-args -p politica:=fifo -p duracion_movimiento_s:=0.2
```

En otra terminal:

```bash
ros2 topic list                      # /arm/queue_state, /joint_states, /move_arm/...
ros2 topic hz /arm/queue_state       # esperado: ~5 Hz
ros2 topic echo /arm/queue_state     # un solo executing_goal_id a la vez
ros2 topic info /joint_states -v     # Publisher count: 1 (arm_broker)
```

Debe cumplirse:

- [ ] `/arm/queue_state` publica a ~5 Hz (aunque no haya goals).
- [ ] Solo el nodo `arm_broker` publica `/joint_states` (antes de lanzar clientes: 0 mensajes).
- [ ] Nunca hay más de un `executing_goal_id`.
- [ ] Varios goals pueden estar en cola a la vez (`queue_length` > 1).

Repetir con `-p politica:=round_robin`.

## 2. Varios clientes en la misma computadora

Una terminal por cliente, todas con el mismo `ROS_DOMAIN_ID`. Para que queden pedidos pendientes
se usa el modo asíncrono y `pausa_s:=0.0`; conviene `repeticiones:=5` para que la cola dure:

```bash
ros2 run arm_broker cliente --ros-args -r __node:=cliente_a -p client_id:=A -p priority:=1 \
  -p modo:=asincrono -p pausa_s:=0.0 -p repeticiones:=5 -p traza:=trazas/prueba.csv
# ... igual para B (priority:=2), C (3) y D (4)
```

Orden esperado de atención (con A, B y C enviando 3 goals, uno tras otro):

```
FIFO:        A1 A2 A3 B1 B2 B3 C1 C2 C3
Round Robin: A1 B1 C1 A2 B2 C2 A3 B3 C3
```

Se ve en `ros2 topic echo /arm/queue_state --field executing_client` o en el log del broker
(`EJECUTANDO <cliente> ...`). Un cliente con una pose fuera de rango debe aparecer como
`RECHAZADA` y en `rechazos.csv`.

## 3. Grabar con `ros2 bag`

```bash
ros2 bag record -o ensayo_fifo /arm/queue_state /joint_states
# lanzar los clientes; al terminar, Ctrl-C
ros2 bag info ensayo_fifo            # deben aparecer los dos tópicos con mensajes
```

Repetir con Round Robin (`ensayo_round_robin`). Los bags de ensayo no se entregan ni se guardan en
`evidencias/`.

## 4. Exportar y analizar

```bash
python3 analisis/exportar_csv.py ensayo_fifo --salida ensayo/fifo
python3 analisis/exportar_csv.py ensayo_round_robin --salida ensayo/round_robin
cp rechazos.csv ensayo/fifo/   # si el broker se lanzó desde esta carpeta (opcional)
python3 analisis/metricas.py ensayo/fifo/queue_state.csv ensayo/round_robin/queue_state.csv \
    --salida ensayo/comparacion.png
```

`metricas.py` debe imprimir para cada política: espera media/p95/máxima (global y por prioridad),
índice de inanición, goals por cliente, Jain (final y a mitad), goals rechazados con sus causas y
violaciones de exclusión mutua (0), y generar la figura.

Con `duracion_movimiento_s:=0.2` el brazo atiende muy rápido y la cola casi no se ve a 5 Hz; para
ensayar las métricas conviene `duracion_movimiento_s:=1.0` o más.

## 5. Corrida completa automatizada

```bash
DURACION_S=1.0 SALIDA=/tmp/ensayo_fifo bash herramientas/experimento_item3.sh fifo
```

(`SALIDA` fuera de `evidencias/` para no mezclar ensayos con evidencia oficial.)
