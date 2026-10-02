#!/usr/bin/env python3
""" MÉTRICAS DEL ÍTEM 3 REALIZADAS A PARTIR DE LOS CSV DEL BAG Y DE LA FIGURA COMPARATIVA """

""" - Convención de prioridad: Un mayor número signica que es más urgente (esto lo declara MoveArm.action). 
Asi que, el índice de inanición es la espera máxima del número MÁS BAJO."""

""" - Uso: Solo un directorio por política, cada uno con los CSV que salen de exportar_csv.py

    python3 metricas.py evidencias/item_3/fifo/queue_state.csv \\
                        evidencias/item_3/round_robin/queue_state.csv \\
                        --salida evidencias/item_3/comparacion_politicas.png

    Junto a cada queue_state.csv se buscan, si existen:
        joint_states.csv: para detectar violaciones de exclusión mutua
        rechazos.csv: para las causas de los goals rechazados (lo escribe el broker)."""

""" - Calcula: Se calcula la espera media, p95 y máxima (global y por prioridad), índice de inanición,
goals atendidos por cliente, equidad de Jain (al final y a la mitad de la corrida), goals
rechazados con sus causas y violaciones de exclusión mutua."""

""" - Limitación de la medición: Se publica a 5 Hz: /arm/queue_state , así que cada espera se
conoce con una resolución de ~ 0.2 s, y un goal que empieza a ejecutarse antes de la
siguiente publicación no llega a verse en la cola. Afecta igual a todas las políticas."""

import argparse
import bisect
import csv
import os
import statistics
import sys

# Ventana alrededor de un mensaje de /joint_states en la que debe verse un goal ejecutándose
VENTANA_EJECUCION_S = 0.25

# 1. Lectura de los CSV
def filas_cola(ruta):
    """Se leen las filas de queue_state.csv con el tiempo en segundos, ordenadas por tiempo"""
    with open(ruta, newline='') as f:
        filas = list(csv.DictReader(f))
    for fila in filas:
        fila['t_s'] = int(fila['t_ns']) / 1e9
    filas.sort(key=lambda r: r['t_s'])
    return filas


def leer(ruta):
    """Devuelve: esperas_por_cliente, esperas_por_prioridad, completados."""
    vistos = {}
    por_cliente = {}
    por_prioridad = {}
    completados = 0

    for fila in filas_cola(ruta):
        ids = [g for g in fila['queued_goal_ids'].split('|') if g]
        clientes = [c for c in fila['queued_clients'].split('|') if c]
        prioridades = [p for p in fila['queued_priorities'].split('|') if p]
        esperas = [w for w in fila['queued_wait_s'].split('|') if w]
        for gid, c, p, w in zip(ids, clientes, prioridades, esperas):
            previo = vistos.get(gid)
            espera = float(w)
            if previo is None or espera > previo[2]:
                vistos[gid] = (c, int(p), espera)
        completados = max(completados, int(fila['total_completed'] or 0))

    for cliente, prioridad, espera in vistos.values():
        por_cliente.setdefault(cliente, []).append(espera)
        por_prioridad.setdefault(prioridad, []).append(espera)
    return por_cliente, por_prioridad, completados


def leer_rechazos(ruta):
    """Se cuentan los rechazos por causa desde rechazos.csv (lo escribe el broker)"""
    causas = {}
    with open(ruta, newline='', encoding='utf-8') as f:
        for fila in csv.DictReader(f):
            causas[fila['causa']] = causas.get(fila['causa'], 0) + 1
    return causas


# 2. Estadísticas
def p95(xs):
    if not xs:
        return 0.0
    ordenados = sorted(xs)
    k = max(0, min(len(ordenados) - 1, int(round(0.95 * (len(ordenados) - 1)))))
    return ordenados[k]


def jain(valores):
    """Equidad de Jain: 1.0 significa reparto perfecto, 1/n significa que uno se lo lleva todo."""
    if not valores or sum(valores) == 0:
        return 0.0
    n = len(valores)
    return (sum(valores) ** 2) / (n * sum(v * v for v in valores))


def jain_a_mitad(filas):
    """Se calcula la equidad de Jain sobre los goals que ya habían empezado a mitad de la corrida"""
    """Al final de una traza finita todos los clientes atendieron lo mismo y Jain vale 1.0 para
    cualquier política, a mitad de la corrida sí se ve quién iba ganando"""
    """Termina retornando: jain, {cliente: goals} o (None, {}) si no hubo ejecuciones"""
    inicios = {}   # goal_id -> (cliente, primer instante en que se vio ejecutándose)
    for fila in filas:
        gid = fila['executing_goal_id']
        if gid and gid not in inicios:
            inicios[gid] = (fila['executing_client'], fila['t_s'])
    if not inicios:
        return None, {}

    t0 = min(t for _, t in inicios.values())
    t1 = max(t for _, t in inicios.values())
    t_mitad = (t0 + t1) / 2
    conteo = {c: 0 for c, _ in inicios.values()}
    for cliente, t in inicios.values():
        if t <= t_mitad:
            conteo[cliente] += 1
    return jain(list(conteo.values())), conteo

# 3. Exclusión mutua
def violaciones_exclusion(filas, ruta_joint):
    """Se buscan indicios de violación de exclusión mutua en los CSV"""
    """Un bag de /joint_states no dice quién publicó cada mensaje, así que se correlaciona con /arm/queue_state 
    (que solo tiene UN executing_goal_id a la vez). 
    
    Se cuentan:
      - intercaladas: un goal que vuelve a ejecutarse después de que otro empezó
      - sin_ejecucion: mensajes de /joint_states sin ningún goal ejecutándose cerca
    
    Es un indicio, no una prueba: la prueba de que no hay otro publicador es
    'ros2 topic info /joint_states -v', que debe mostrar un solo publicador (el broker)"""
    
    """Se retorna un dict con "intercaladas", "sin_ejecucion" o None si no hay joint_states.csv y 'mensajes_joint'"""
    # Goals intercalados: la secuencia de executing_goal_id no puede repetir un goal ya cerrado
    secuencia = []
    for fila in filas:
        gid = fila['executing_goal_id']
        if gid and (not secuencia or secuencia[-1] != gid):
            secuencia.append(gid)
    intercaladas = len(secuencia) - len(set(secuencia))

    resultado = {'intercaladas': intercaladas, 'sin_ejecucion': None, 'mensajes_joint': 0}
    if not ruta_joint or not os.path.isfile(ruta_joint) or not filas:
        return resultado

    # Instantes en el que /arm/queue_state vio un goal ejecutandose
    tiempos = [f['t_s'] for f in filas]
    ejecutando = [f['t_s'] for f in filas if f['executing_goal_id']]

    sin_ejecucion = 0
    mensajes = 0
    with open(ruta_joint, newline='') as f:
        for fila in csv.DictReader(f):
            t = int(fila['t_ns']) / 1e9
            # Solo se juzgan mensajes dentro del intervalo cubierto por /arm/queue_state
            if t < tiempos[0] - VENTANA_EJECUCION_S or t > tiempos[-1] + VENTANA_EJECUCION_S:
                continue
            mensajes += 1
            i = bisect.bisect_left(ejecutando, t - VENTANA_EJECUCION_S)
            if i >= len(ejecutando) or ejecutando[i] > t + VENTANA_EJECUCION_S:
                sin_ejecucion += 1
    resultado['sin_ejecucion'] = sin_ejecucion
    resultado['mensajes_joint'] = mensajes
    return resultado


# 4. Resumen por política
def resumen(nombre, ruta):
    por_cliente, por_prioridad, completados = leer(ruta)
    todas = [w for ws in por_cliente.values() for w in ws]

    print(f'\n=== {nombre}   ({ruta})')
    if not todas:
        print('  sin datos')
        return None

    filas = filas_cola(ruta)
    carpeta = os.path.dirname(os.path.abspath(ruta))
    ruta_joint = os.path.join(carpeta, 'joint_states.csv')
    ruta_rechazos = os.path.join(carpeta, 'rechazos.csv')

    print(f'  pedidos observados : {len(todas)}')
    print(f'  completados        : {completados}')
    print(f'  espera media       : {statistics.mean(todas):.2f} s')
    print(f'  espera p95         : {p95(todas):.2f} s')
    print(f'  espera máxima      : {max(todas):.2f} s')

    print('  por prioridad:')
    for pr in sorted(por_prioridad):
        ws = por_prioridad[pr]
        print(f'    prioridad {pr}: n={len(ws):3d}  media={statistics.mean(ws):6.2f}s  '
              f'p95={p95(ws):6.2f}s  máx={max(ws):6.2f}s')

    peor = min(por_prioridad) if por_prioridad else None
    inanicion = max(por_prioridad[peor]) if peor is not None else 0.0
    print(f'  índice de inanición: {inanicion:.2f} s  '
          f'(espera máxima de prioridad {peor}, la menos urgente)')

    atendidos = {c: len(ws) for c, ws in sorted(por_cliente.items())}
    print(f'  goals por cliente  : {atendidos}')
    j = jain(list(atendidos.values()))
    print(f'  equidad de Jain    : {j:.3f}  (al final de la corrida)')
    jm, en_mitad = jain_a_mitad(filas)
    if jm is not None:
        print(f'  Jain a mitad       : {jm:.3f}  (goals ya iniciados a mitad: {en_mitad})')

    # Goals rechazados: cantidad (queue_state) y causas (rechazos.csv del broker)
    n_rechazados = max(int(f['total_rejected'] or 0) for f in filas)
    n_aceptados = max(int(f['total_accepted'] or 0) for f in filas)
    causas = leer_rechazos(ruta_rechazos) if os.path.isfile(ruta_rechazos) else None
    print(f'  goals aceptados : {n_aceptados}')
    print(f'  goals rechazados : {n_rechazados}   causas: ' 
          f'{causas if causas is not None else "sin rechazos.csv junto al queue_state.csv"}')

    # Violaciones de exclusión mutua: deben ser 0
    v = violaciones_exclusion(filas, ruta_joint)
    if v['sin_ejecucion'] is None:
        print(f'  violaciones excl.  : intercaladas={v["intercaladas"]}  '
              f'(sin joint_states.csv no se revisan los mensajes)')
    else:
        print(f'  violaciones excl.  : intercaladas={v["intercaladas"]}  '
              f'joint_states sin goal ejecutando={v["sin_ejecucion"]} '
              f'de {v["mensajes_joint"]} mensajes   (deben ser 0)')

    return {'nombre': nombre, 'todas': todas, 'por_prioridad': por_prioridad, 'atendidos': atendidos, 
            'inanicion': inanicion, 'jain': j, 'jain_mitad': jm, 'violaciones': v, 'rechazados': n_rechazados}

# 5. Figura comparativa

# Paleta categórica en orden fijo: Significa que cada política conserva su color en todos los paneles
COLORES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']   # azul, naranja, aqua, amarillo
TINTA, TINTA_2, TINTA_3 = '#0b0b0b', '#52514e', '#898781'
SUPERFICIE, REJILLA, EJE = '#fcfcfb', '#e1e0d9', '#c3c2b7'


def figura(datos, salida='comparacion_politicas.png'):
    """Se dibuja una sola figura de cuatro paneles que compara las políticas"""
    """Paneles: espera media y p95 · p95 por prioridad · índice de inanición · equidad de Jain"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nSin matplotlib; no genero la figura.  pip install matplotlib')
        return

    nombres = [d['nombre'] for d in datos]
    color = {n: COLORES[i % len(COLORES)] for i, n in enumerate(nombres)}

    fig, ejes = plt.subplots(2, 2, figsize=(11.5, 8), facecolor=SUPERFICIE)
    fig.suptitle('Comparación de políticas de cola: ' + ' vs '.join(nombres),
                 color=TINTA, fontsize=13, x=0.06, ha='left')

    def preparar(ax, titulo, ylabel):
        ax.set_facecolor(SUPERFICIE)
        ax.set_title(titulo, color=TINTA, fontsize=11, loc='left')
        ax.set_ylabel(ylabel, color=TINTA_2)
        ax.tick_params(colors=TINTA_2, length=0)
        for lado in ('top', 'right', 'left'):
            ax.spines[lado].set_visible(False)
        ax.spines['bottom'].set_color(EJE)
        ax.yaxis.grid(True, color=REJILLA, linewidth=0.8)
        ax.set_axisbelow(True)

    def barras(ax, etiquetas, series, formato='{:.1f}'):
        """series = política: [valor por etiqueta], una barra fina por política"""
        ancho = 0.8 / max(1, len(series))
        for k, (nombre, valores) in enumerate(series.items()):
            xs = [i + (k - (len(series) - 1) / 2) * ancho for i in range(len(etiquetas))]
            ax.bar(xs, valores, ancho * 0.9, label=nombre, color=color[nombre],
                   edgecolor=SUPERFICIE, linewidth=1.5)
            for x, v in zip(xs, valores):
                ax.text(x, v, formato.format(v), ha='center', va='bottom',
                        fontsize=8, color=TINTA_2)
        ax.set_xticks(range(len(etiquetas)))
        ax.set_xticklabels(etiquetas)
        ax.margins(y=0.15)

    # Panel 1: espera media y p95 (global)
    ax = ejes[0][0]
    preparar(ax, 'Espera global en cola', 'espera (s)')
    barras(ax, ['media', 'p95', 'máxima'],
           {d['nombre']: [statistics.mean(d['todas']), p95(d['todas']), max(d['todas'])]
            for d in datos})
    ax.legend(frameon=False, labelcolor=TINTA_2, fontsize=9)

    # Panel 2: p95 por prioridad
    ax = ejes[0][1]
    preparar(ax, 'p95 de espera por prioridad', 'espera p95 (s)')
    prioridades = sorted({p for d in datos for p in d['por_prioridad']})
    barras(ax, [f'prioridad {p}' for p in prioridades],
           {d['nombre']: [p95(d['por_prioridad'].get(p, [])) for p in prioridades]
            for d in datos})

    # Panel 3: índice de inanición
    ax = ejes[1][0]
    preparar(ax, 'Índice de inanición\n(espera máxima de la prioridad más baja)', 'espera (s)')
    barras(ax, [''], {d['nombre']: [d['inanicion']] for d in datos})
    ax.set_xlim(-1.3, 1.3)   # Una sola categoría: barras finas, no de ancho completo

    # Panel 4: equidad de Jain, al final y a mitad de la corrida
    ax = ejes[1][1]
    preparar(ax, 'Equidad de Jain (1.0 = reparto perfecto)', 'índice de Jain')
    barras(ax, ['al final', 'a mitad de la corrida'],
           {d['nombre']: [d['jain'], d['jain_mitad'] if d['jain_mitad'] is not None else 0.0]
            for d in datos}, formato='{:.3f}')
    ax.set_ylim(0, 1.15)
    ax.axhline(1.0, color=TINTA_3, linewidth=0.8, linestyle='--')

    violaciones = sum(d['violaciones']['intercaladas'] + (d['violaciones']['sin_ejecucion'] or 0)
                      for d in datos)
    fig.text(0.06, 0.012,
             f'Violaciones de exclusión mutua detectadas en los CSV: {violaciones} · '
             f'goals rechazados: ' + ', '.join(f'{d["nombre"]}={d["rechazados"]}' for d in datos),
             color=TINTA_2, fontsize=9)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(salida, dpi=150, facecolor=SUPERFICIE)
    print(f'\nFigura: {salida}')


# 6. Punto de entrada
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+', help='un queue_state.csv por política')
    ap.add_argument('--salida', default='comparacion_politicas.png')
    args = ap.parse_args()

    datos = []
    for ruta in args.csvs:
        if not os.path.isfile(ruta):
            sys.exit(f'No encuentro {ruta}')
        r = resumen(os.path.basename(os.path.dirname(os.path.abspath(ruta))) or ruta, ruta)
        if r:
            datos.append(r)

    if len(datos) >= 2:
        figura(datos, args.salida)
    elif datos:
        print('\nCon un solo CSV no hay existe una comparación. Entonces es  necesario grabar una corrida por política.')


if __name__ == '__main__':
    main()

