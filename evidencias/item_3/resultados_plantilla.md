# Ítem 3 — Resultados (PLANTILLA: copiar a `resultados.md` y completar DESPUÉS de las corridas)

Los valores salen de `python3 analisis/metricas.py evidencias/item3/fifo/queue_state.csv evidencias/item3/round_robin/queue_state.csv --salida evidencias/item3/comparacion_politicas.png`.
La columna «Predicho» se copia de `docs/diseño_previo.md` **sin cambiarla**.

## Condiciones de las corridas

| Dato | FIFO | Round Robin |
|---|---|---|
| Fecha y hora | | |
| Commit (`protocolo.txt`) | | |
| Traza y `sha256` (deben ser iguales) | | |
| Goals enviados por cliente | | |
| Publicadores de `/joint_states` (`publicadores_joint_states.txt`) | | |
| Mensajes en el bag (`bag_info.txt`) | | |

## Métricas

| Métrica | Criterio | FIFO predicho | FIFO medido | RR predicho | RR medido |
|---|---|---|---|---|---|
| Violaciones de exclusión mutua | Debe ser 0 | 0 | | 0 | |
| Espera media (global) | | | | | |
| Espera p95 (global) | | | | | |
| Espera máxima (global) | | | | | |
| Índice de inanición | Máx. espera de la prioridad más baja | | | | |
| Equidad de Jain (al final) | Goals atendidos por cliente | 1.000 | | 1.000 | |
| Equidad de Jain (a mitad) | | | | | |
| Goals rechazados (cantidad) | Cantidad y causas | 0 | | 0 | |
| Causas de rechazo | | ninguna | | ninguna | |

### Espera por prioridad (s)

| Prioridad (cliente) | FIFO media | FIFO p95 | FIFO máx | RR media | RR p95 | RR máx |
|---|---|---|---|---|---|---|
| 1 (A) | | | | | | |
| 2 (B) | | | | | | |
| 3 (C) | | | | | | |
| 4 (D) | | | | | | |

## Predicción frente a lo obtenido

| Punto de la predicción | ¿Se cumplió? | Diferencia y explicación |
|---|---|---|
| El orden de atención fue el previsto | | |
| La espera media global fue igual en ambas | | |
| p95 global casi igual | | |
| Round Robin igualó las esperas medias por cliente | | |
| FIFO favoreció a los clientes que llegaron primero | | |
| Jain a mitad de corrida: FIFO < RR | | |
| Inanición: mejor en FIFO que en RR con este orden de llegada | | |

## Conclusión (una frase que sostenga la figura)

`[completar]`
