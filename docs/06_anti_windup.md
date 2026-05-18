# 6. Saturación del Actuador y Anti-Windup

> Aplicación: control de **posición angular** del motor BDC con modelo **3×3** y los **parámetros de la planta real**, considerando la saturación del puente H ($V_{\max} = \pm 24\,\text{V}$).

## 6.1 El problema: el actuador no es ideal

Todo actuador real (puente H, driver) tiene límites: por ejemplo, el motor BDC del banco trabaja con $V_{\max} = +24\,\text{V}$ y $V_{\min} = -24\,\text{V}$.

Cuando el PID solicita una acción mayor (típicamente al inicio de un escalón grande), el voltaje aplicado **se satura** y el motor responde a la velocidad máxima posible. Mientras tanto, el **error sigue siendo positivo**, y el integrador del PID sigue acumulando:

$$ u_i[k] = u_i[k-1] + K_i \cdot T_s \cdot e[k] $$

Esa carga acumulada es el **windup integral**: cuando finalmente la salida llega cerca de la referencia, el integrador ya tiene un valor enorme y produce un sobreimpulso violento; tarda mucho en "descargarse" porque debe integrar error de signo opuesto.

## 6.2 Síntomas

- Sobreimpulso muy superior al diseñado.
- Tiempo de asentamiento mucho mayor.
- Oscilaciones lentas alrededor de la referencia.
- En sistemas críticos: **inestabilidad práctica**.

## 6.3 Solución: Anti-Windup por *Back-Calculation*

Idea: cuando la salida del PID se satura, "frenamos" la integración añadiendo un término que descarga el integrador en proporción a la diferencia entre lo solicitado y lo realmente entregado.

$$ u_{\text{sat}}[k] = \mathrm{sat}\!\big(u_p[k] + u_i[k] + u_d[k],\, V_{\min},\, V_{\max}\big) $$

$$ u_i[k] = u_i[k-1] + K_i \cdot T_s \cdot e[k] + K_{aw} \cdot T_s \cdot \big(u_{\text{sat}}[k-1] - u_{\text{calc}}[k-1]\big) $$

donde $K_{aw}$ es la **ganancia de back-calculation**. Una elección razonable:

$$ K_{aw} = K_p / 10 \quad \text{o} \quad K_{aw} = \sqrt{K_i \cdot K_d} $$

Cuando no hay saturación, $u_{\text{sat}}=u_{\text{calc}}$ y el término extra es cero → el PID funciona normalmente.

### Derivación del valor de $K_{aw}$ por constante de tiempo de descarga

Definiendo $\Delta u[k] = u_{\text{sat}}[k] - u_{\text{calc}}[k] \cdot \le 0$ durante saturación positiva, y suponiendo que durante un transitorio largo el integrador domina ($u \approx u_i$), la recurrencia se aproxima por una **descarga de primer orden** equivalente a la EDO

$$ \dot u_i = -K_{aw} \cdot u_i + \text{(términos de error)}. $$

La **constante de tiempo de descarga** del integrador en saturación es $T_{aw} = 1/K_{aw}$. La heurística

$$ K_{aw} = \sqrt{K_i \cdot K_d} $$

proviene de imponer $T_{aw}$ como **media geométrica** entre las constantes de tiempo natural del integrador ($T_i = K_p/K_i$) y del derivador ($T_d = K_d/K_p$); equilibra la respuesta entre "descargar muy rápido" (oscilaciones secundarias) y "descargar muy lento" (sobreimpulso residual).

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

## 6.7 Estimación del tiempo de saturación

Dada una referencia escalón $\theta_{\text{ref}}$, el motor satura mientras la corriente que pueda entregar el driver no sea suficiente para alcanzar la aceleración demandada. Una cota inferior del **tiempo en saturación** $t_{\text{sat}}$ usando momentum angular:

$$ J_e \cdot \omega_{\max} \approx K \cdot i_{\max} \cdot t_{\text{sat}} - B_e \cdot \omega_{\max} \cdot t_{\text{sat}} $$

con $i_{\max} = V_{\max}/R_a$ (corriente máxima estacionaria) y $\omega_{\max} \approx K_{dc} \cdot V_{\max}$ (velocidad de vacío). Para el motor del banco ($V_{\max}=24\ \text{V}$, $R_a=11$, $K=0.0014$, $J_e = 7.56 \times 10^{-4}$):

$$ i_{\max} = \frac{24}{11} \approx 2.18\,\text{A},\quad \omega_{\max} \approx \frac{V_{\max}}{K_b} = \frac{24}{0.0014} \approx 17.1 \times 10^3\,\text{rad/s}. $$

Si la referencia es $\theta_{\text{ref}} = 10\ \text{rad}$, alcanzarla con velocidad máxima requiere al menos $\theta_{\text{ref}}/\omega_{\max} \approx 0.6\ \text{ms}$ de "movimiento puro" — típicamente despreciable frente al transitorio eléctrico. Es decir, en este motor la saturación se debe casi exclusivamente al **arranque** (rampear $i_a$ contra $L_a$), no a la velocidad terminal.

Esta observación explica por qué el script del capítulo, con $t_p = 5\ \text{s}$, satura **brevemente al inicio** y luego sigue la trayectoria ideal: el problema del windup aquí es transitorio, no permanente.

## 6.8 Ejemplo numérico

Con $K_p = 0.1$ y $K_i = 0.01$ (valores típicos del script con $t_p = 5$): si la salida queda saturada $1\ \text{s}$ con error promedio $5\ \text{rad}$, sin anti-windup el integrador acumula

$$ \Delta u_i = K_i \cdot 5 \cdot 1 = 0.05. $$

Parece poco, pero si el escalón es de $10\ \text{rad}$ y $K_i$ es $10\times$ mayor por un sintonizado más agresivo, $\Delta u_i$ supera fácilmente los 5 V, comparables a la mitad de la saturación. Esa es la "munición" que tarda en gastarse y produce sobreimpulso. Con $K_{aw} = K_p/10 = 0.01$, el integrador se descarga con $T_{aw} = 1/K_{aw} = 100\ \text{s}$—¡demasiado lento! Para este caso conviene la otra heurística:

$$ K_{aw} = \sqrt{K_i K_d} \approx \sqrt{0.01 \cdot 0.1} \approx 0.0316,\quad T_{aw} \approx 31.6\,\text{s}. $$

La lección: ambas heurísticas son puntos de partida; siempre conviene **simular el lazo no lineal** (como hace el script) y ajustar $K_{aw}$ para que la descarga del integrador encaje con la dinámica de la planta.
