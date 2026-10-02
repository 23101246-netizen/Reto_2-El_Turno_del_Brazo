#!/usr/bin/env bash
# Corre UNA política del ítem 3 con el protocolo fijo y deja la evidencia en evidencias/item_3/<política>/
#
#   bash herramientas/experimento_item3.sh fifo
#   bash herramientas/experimento_item3.sh round_robin
#
# La única diferencia entre las dos corridas es `politica:=`. Todo lo demás sale de las variables de
# abajo, que se guardan en protocolo.txt para poder demostrar que fueron iguales.
#
# Requisitos: ROS 2 Humble y el workspace ya con `source` (ros2 run arm_broker broker debe existir),
# y ROS_DOMAIN_ID propio. Este script NO arranca el driver del brazo: con o sin robot, el broker
# publica /joint_states igual.
#
# Variables (con su valor por defecto):
#   TRAZA        CSV de poses; la traza OFICIAL del docente en la corrida real
#   CLIENTES     "A:1 B:2 C:3 D:4"  nombre:prioridad, en el orden en que arrancan
#   REPETICIONES 1
#   PAUSA_S      0.0   pausa entre envíos de cada cliente (0 = todo seguido)
#   DURACION_S   3.0   duracion_movimiento_s del broker (0.2 sirve para ensayar rápido)
#   PASOS        10    pasos_interpolacion del broker
#   ESCALON_S    0.5   separación entre el arranque de un cliente y el siguiente
#   ARRANQUE_S   8     margen para que todos los `ros2 run` estén listos antes del primer envío
#   SALIDA       evidencias/item_3/<política>

set -euo pipefail

# Con control de trabajos cada proceso en segundo plano lleva su propio grupo y recibe SIGINT con
# normalidad; sin él, bash los lanza ignorando SIGINT y `kill -INT` no detendría el bag ni el broker
set -m

POLITICA="${1:-}"
case "$POLITICA" in
  fifo|round_robin) ;;
  *) echo "uso: $0 fifo|round_robin" >&2; exit 2 ;;
esac

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRAZA="${TRAZA:-$RAIZ/trazas/prueba.csv}"
CLIENTES="${CLIENTES:-A:1 B:2 C:3 D:4}"
REPETICIONES="${REPETICIONES:-1}"
PAUSA_S="${PAUSA_S:-0.0}"
DURACION_S="${DURACION_S:-3.0}"
PASOS="${PASOS:-10}"
ESCALON_S="${ESCALON_S:-0.5}"
ARRANQUE_S="${ARRANQUE_S:-8}"
SALIDA="${SALIDA:-$RAIZ/evidencias/item_3/$POLITICA}"

# 1. Comprobaciones previas
command -v ros2 >/dev/null || { echo "ros2 no está en el PATH: source /opt/ros/humble/setup.bash" >&2; exit 1; }
ros2 pkg prefix arm_broker >/dev/null 2>&1 || { echo "arm_broker no está compilado: source install/setup.bash" >&2; exit 1; }
[ -f "$TRAZA" ] || { echo "no encuentro la traza: $TRAZA" >&2; exit 1; }
: "${ROS_DOMAIN_ID:?exporte ROS_DOMAIN_ID (42 + n.º de equipo)}"

# La evidencia no se sobreescribe: si ya hay resultados, se mueven o se borran a mano
if [ -d "$SALIDA" ] && [ -n "$(find "$SALIDA" -mindepth 1 ! -name .gitkeep -print -quit)" ]; then
  echo "ya hay resultados en $SALIDA; muévalos antes de repetir la corrida" >&2
  exit 1
fi
mkdir -p "$SALIDA"

# Se envía SIGINT (Ctrl-C) al grupo completo: `ros2 run` y el nodo que lanza
detener() {
  kill -INT -- "-$1" 2>/dev/null || kill -INT "$1" 2>/dev/null || true
}

PIDS=()
limpiar() {
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && detener "$pid"
  done
}
trap limpiar EXIT

# 2. Se guarda el protocolo tal como se ejecuta
cat > "$SALIDA/protocolo.txt" <<PROTO
politica=$POLITICA
fecha=$(date -Is)
traza=$TRAZA
traza_sha256=$(sha256sum "$TRAZA" | cut -d' ' -f1)
poses_en_traza=$(grep -cv -e '^[[:space:]]*#' -e '^[[:space:]]*$' "$TRAZA")
clientes=$CLIENTES
repeticiones=$REPETICIONES
modo=asincrono
pausa_s=$PAUSA_S
duracion_movimiento_s=$DURACION_S
pasos_interpolacion=$PASOS
escalon_s=$ESCALON_S
ros_domain_id=$ROS_DOMAIN_ID
topicos_grabados=/arm/queue_state /joint_states
commit=$(git -C "$RAIZ" rev-parse --short HEAD 2>/dev/null || echo desconocido)
PROTO

# 3. Broker con la política a medir
echo "[1/6] broker (politica=$POLITICA)"
ros2 run arm_broker broker --ros-args \
  -p politica:="$POLITICA" \
  -p duracion_movimiento_s:="$DURACION_S" \
  -p pasos_interpolacion:="$PASOS" \
  -p archivo_rechazos:="$SALIDA/rechazos.csv" \
  > "$SALIDA/broker.log" 2>&1 &
BROKER=$!
PIDS+=("$BROKER")
sleep 3

# 4. Evidencia de publicador único y grabación
echo "[2/6] publicadores de /joint_states y grabación del bag"
ros2 topic info /joint_states -v > "$SALIDA/publicadores_joint_states.txt" 2>&1 || true
ros2 bag record -o "$SALIDA/bag" /arm/queue_state /joint_states > "$SALIDA/bag.log" 2>&1 &
BAG=$!
PIDS+=("$BAG")
sleep 2

# 5. Clientes: todos arrancan en instantes fijos, escalonados y en el orden de CLIENTES
echo "[3/6] clientes ($CLIENTES), escalón ${ESCALON_S}s"
INICIO="$(python3 -c "import time; print(time.time() + $ARRANQUE_S)")"
CLIENTE_PIDS=()
i=0
for par in $CLIENTES; do
  nombre="${par%%:*}"
  prioridad="${par##*:}"
  inicio_cliente="$(python3 -c "print($INICIO + $i * $ESCALON_S)")"
  ros2 run arm_broker cliente --ros-args \
    -r __node:="cliente_$nombre" \
    -p client_id:="$nombre" \
    -p priority:="$prioridad" \
    -p traza:="$TRAZA" \
    -p repeticiones:="$REPETICIONES" \
    -p modo:=asincrono \
    -p pausa_s:="$PAUSA_S" \
    -p inicio_unix:="$inicio_cliente" \
    > "$SALIDA/cliente_$nombre.log" 2>&1 &
  CLIENTE_PIDS+=("$!")
  PIDS+=("$!")
  i=$((i + 1))
done

echo "[4/6] esperando a que terminen los clientes (puede tardar varios minutos)"
for pid in "${CLIENTE_PIDS[@]}"; do
  wait "$pid" || echo "un cliente terminó con error; revise los cliente_*.log" >&2
done
sleep 2

# 6. Se detiene la grabación y el broker
echo "[5/6] deteniendo bag y broker"
detener "$BAG"
wait "$BAG" 2>/dev/null || true
detener "$BROKER"
wait "$BROKER" 2>/dev/null || true
PIDS=()

ros2 bag info "$SALIDA/bag" > "$SALIDA/bag_info.txt" 2>&1 || true

# 7. Exportación a CSV
echo "[6/6] exportando a CSV"
python3 "$RAIZ/analisis/exportar_csv.py" "$SALIDA/bag" --salida "$SALIDA"

echo
echo "Listo: $SALIDA"
echo "Cuando estén las dos políticas:"
echo "  python3 analisis/metricas.py evidencias/item_3/fifo/queue_state.csv \\"
echo "      evidencias/item_3/round_robin/queue_state.csv --salida evidencias/item_3/comparacion_politicas.png"
