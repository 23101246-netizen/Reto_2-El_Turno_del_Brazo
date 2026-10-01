# Cierre reflexivo — ¿Qué política de cola llevaríamos a CapyTown y por qué?

> **BORRADOR (máximo una página).** Los campos `[ ]` se completan con los resultados de las corridas
> oficiales (`evidencias/item3/resultados.md`). No se cambia la predicción del documento de diseño previo.

**Equipo:** `[ ]` · **Fecha:** `[ ]`

## 1. Qué medimos

Comparamos FIFO y Round Robin con la misma traza, los mismos cuatro clientes y prioridades, el modo
asíncrono, la misma duración e interpolación y el mismo procedimiento de inicio; solo cambió
`politica:=`. Registramos `/arm/queue_state` y `/joint_states` con `ros2 bag`.

## 2. Qué esperábamos y qué obtuvimos

| Métrica | Predicho FIFO / RR | Medido FIFO / RR | ¿Coincide? |
|---|---|---|---|
| Violaciones de exclusión mutua | 0 / 0 | `[ ]` / `[ ]` | `[ ]` |
| Espera media / p95 global | 58.7 s · 111 s / 58.7 s · 112 s (caso de referencia; recalcular con la traza real) | `[ ]` | `[ ]` |
| p95 por prioridad | `[ ]` | `[ ]` | `[ ]` |
| Índice de inanición | 27 s / 110 s | `[ ]` / `[ ]` | `[ ]` |
| Jain al final / a mitad | 1.000 · 0.5 / 1.000 · 1.0 | `[ ]` | `[ ]` |
| Rechazos (cantidad y causa) | 0 / 0 | `[ ]` | `[ ]` |

Donde la medición se aparta de la predicción, la explicación es: `[ ]`.

## 3. Inanición

Espera máxima de la prioridad más baja en cada política y qué la explica (orden de llegada, ráfagas): `[ ]`.

## 4. Decisión

Llevaríamos a CapyTown **`[ FIFO / Round Robin / otra ]`** porque `[ ]`. Criterios a pesar con nuestros datos:
si importa más la igualdad entre visitantes o la urgencia, si pesa más la espera máxima que la media
(la media global no cambió entre políticas), y si un cliente con ráfagas puede acaparar el brazo.

## 5. Limitaciones

Resolución de 0.2 s (5 Hz), una corrida por política, traza finita (Jain final siempre 1.000): `[ ]`.
