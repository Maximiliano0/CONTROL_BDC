# 6. Saturación del Actuador y Anti-Windup

> Aplicación: control de **posición angular** del motor BDC con modelo **3×3** y los **parámetros de la planta real**, considerando la saturación del puente H ($V_{\max} = \pm 24\,\text{V}$).

## 6.1 El problema: el actuador no es ideal

Todo actuador real (puente H, driver) tiene límites: por ejemplo, el motor BDC del banco trabaja con $V_{\max} = +24\,\text{V}$ y $V_{\min} = -24\,\text{V}$.

Cuando el PID solicita una acción mayor (típicamente al inicio de un escalón grande), el voltaje aplicado **se satura** y el motor responde a la velocidad máxima posible. Mientras tanto, el **error sigue siendo positivo**, y el integrador del PID sigue acumulando:

$$ u_i[k] = u_i[k-1] + K_i\, T_s\, e[k] $$

Esa carga acumulada es el **windup integral**: cuando finalmente la salida llega cerca de la referencia, el integrador ya tiene un valor enorme y produce un sobreimpulso violento; tarda mucho en "descargarse" porque debe integrar error de signo opuesto.

## 6.2 Síntomas

- Sobreimpulso muy superior al diseñado.
- Tiempo de asentamiento mucho mayor.
- Oscilaciones lentas alrededor de la referencia.
- En sistemas críticos: **inestabilidad práctica**.

## 6.3 Solución: Anti-Windup por *Back-Calculation*

Idea: cuando la salida del PID se satura, "frenamos" la integración añadiendo un término que descarga el integrador en proporción a la diferencia entre lo solicitado y lo realmente entregado.

$$ u_{\text{sat}}[k] = \mathrm{sat}\!\big(u_p[k] + u_i[k] + u_d[k],\, V_{\min},\, V_{\max}\big) $$

$$ u_i[k] = u_i[k-1] + K_i\,T_s\,e[k] + K_{aw}\,T_s\,\big(u_{\text{sat}}[k-1] - u_{\text{calc}}[k-1]\big) $$

donde $K_{aw}$ es la **ganancia de back-calculation**. Una elección razonable:

$$ K_{aw} = K_p / 10 \quad \text{o} \quad K_{aw} = \sqrt{K_i\,K_d} $$

Cuando no hay saturación, $u_{\text{sat}}=u_{\text{calc}}$ y el término extra es cero → el PID funciona normalmente.

## 6.4 Otras estrategias

- **Clamping (conditional integration):** congelar el integrador mientras la salida esté saturada y el error tenga el mismo signo.
- **Integración inversa:** restar la diferencia saturado-calculado directamente del estado integral.
- **Tracking anti-windup:** dos lazos, uno externo y otro interno de seguimiento de la acción real.

## 6.5 Validación

El script [pid_windup_z.m](../06_anti_windup/pid_windup_z.m) implementa **manualmente** (sin `pidtune` en simulación) el lazo no lineal con saturación sobre el modelo 3×3 de posición y compara, en tres subgráficos sincronizados:

1. **Posición:** referencia vs. PID con windup vs. PID con anti-windup.
2. **Esfuerzo de control:** $u_{\text{calc}}$, $u_{\text{sat}}$ con bandas grises mostrando la zona prohibida.
3. **Memoria integral $u_i[k]$:** evidencia gráficamente cómo el integrador sin AW se "carga" más allá de lo necesario.

Es un script **paramétrico** (`Mp`, `tp`, `Escalon_Ref`) que reescala automáticamente los ejes para usar en clase con distintos escenarios.

## 6.6 Material

- [pid_windup_z.m](../06_anti_windup/pid_windup_z.m)
