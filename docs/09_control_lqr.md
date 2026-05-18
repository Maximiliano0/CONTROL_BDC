# 9. Control LQR Discreto (Cuadrático Óptimo)

> Aplicación: control de **posición angular** del motor BDC con modelo **3×3** discretizado por ZOH y los **parámetros de la planta real**, eligiendo la realimentación que **minimiza un funcional de costo cuadrático**.

## 9.1 Motivación

En los capítulos 02 y 07 ubicamos los polos del lazo cerrado "a mano" a partir de especificaciones temporales ($M_p$, $t_p$). Ese enfoque tiene tres limitaciones:

1. **No considera el esfuerzo de control**. Si los polos elegidos son muy rápidos, el voltaje pedido puede ser enorme y saturar el actuador.
2. **No escala bien con $n$**. En sistemas MIMO o de orden alto elegir $n$ polos consistentes con todas las especificaciones es un arte.
3. **No es óptimo** en ningún sentido formal.

El **regulador lineal cuadrático** (Linear Quadratic Regulator, **LQR**) reformula el problema: en lugar de elegir polos, el diseñador elige **pesos** sobre los estados y sobre la entrada, y el algoritmo entrega la ley $u = -K\,x$ que minimiza un funcional cuadrático. Los polos del lazo cerrado **salen como consecuencia** y siempre quedan dentro del círculo unitario (estabilidad garantizada).

## 9.2 Funcional de Costo Discreto

Para el sistema $x[k+1] = \Phi\,x[k] + \Gamma\,u[k]$ se define el costo de horizonte infinito:

$$
\boxed{\;J = \sum_{k=0}^{\infty}\Bigl( x[k]^T\,Q\,x[k] + u[k]^T\,R\,u[k] \Bigr)\;}
$$

con:

- $Q \in \mathbb{R}^{n\times n}$, **simétrica semidefinida positiva** ($Q \succeq 0$): penaliza desviaciones de los estados respecto del origen.
- $R \in \mathbb{R}^{m\times m}$, **simétrica definida positiva** ($R \succ 0$): penaliza el esfuerzo de control.

Interpretación:

- $Q$ grande → se persiguen los estados con prisa → respuesta rápida, mucho voltaje.
- $R$ grande → cualquier voltaje "duele" en $J$ → respuesta suave, poco voltaje.

## 9.3 Solución Óptima — Ecuación de Riccati Discreta

La ley de control óptima es **lineal y estacionaria**:

$$
u[k] = -K\,x[k]
$$

con

$$
K = \bigl(R + \Gamma^T P\,\Gamma\bigr)^{-1}\,\Gamma^T P\,\Phi,
$$

donde $P$ es la **única solución simétrica definida positiva** de la **Ecuación Algebraica de Riccati Discreta** (DARE):

$$
P = \Phi^T P\,\Phi \;-\; \Phi^T P\,\Gamma\,\bigl(R + \Gamma^T P\,\Gamma\bigr)^{-1}\,\Gamma^T P\,\Phi \;+\; Q.
$$

En MATLAB se resuelve con una sola línea:

```matlab
[K, P, eig_cl] = dlqr(Phi, Gamma, Q, R);
```

`dlqr` devuelve $K$, la matriz $P$ y los autovalores del lazo cerrado $\Phi - \Gamma K$ (todos con $|z|<1$ por construcción).

### Derivación por programación dinámica (Bellman)

Definiendo la **función de valor** $V_k(x) = x^T P_k x$ (forma cuadrática por la naturaleza del costo), Bellman a tiempo inverso establece:

$$ V_k(x) = \min_u\Bigl\{ x^T Q x + u^T R u + V_{k+1}(\Phi x + \Gamma u) \Bigr\}. $$

Sustituyendo $V_{k+1} = x^T P_{k+1} x$, expandiendo y derivando respecto a $u$:

$$ \frac{\partial}{\partial u}\bigl[u^T R u + (\Phi x + \Gamma u)^T P_{k+1}(\Phi x + \Gamma u)\bigr] = 2 R u + 2 \Gamma^T P_{k+1}(\Phi x + \Gamma u) = 0. $$

Despejando:

$$ u^* = -\bigl(R + \Gamma^T P_{k+1}\Gamma\bigr)^{-1}\Gamma^T P_{k+1}\Phi\,x = -K_k\,x. $$

Reinsertando $u^*$ en la igualdad de Bellman se obtiene la **recurrencia de Riccati**:

$$ P_k = Q + \Phi^T P_{k+1}\Phi - \Phi^T P_{k+1}\Gamma\bigl(R + \Gamma^T P_{k+1}\Gamma\bigr)^{-1}\Gamma^T P_{k+1}\Phi. $$

En régimen estacionario ($N \to \infty$, $P_k \to P$) se recupera exactamente la DARE.

## 9.4 Condiciones de Existencia

Para que el LQR tenga solución única y estabilizadora hace falta:

| Condición | Significado |
|-----------|-------------|
| $(\Phi, \Gamma)$ **estabilizable** | Todo modo inestable es controlable (si todo el sistema es controlable, se cumple). |
| $(\Phi, \sqrt{Q})$ **detectable** | Todo modo inestable se "ve" a través de $Q$ (basta con $Q \succ 0$). |
| $R \succ 0$ | El esfuerzo de control siempre se penaliza estrictamente. |

Para el motor BDC 3×3 con los parámetros reales se cumplen las tres.

## 9.5 Elección de $Q$ y $R$ — Regla de Bryson

La forma más sistemática de elegir $Q$ y $R$ es la **regla de Bryson**: tomar los valores **máximos tolerables** de cada estado y de la entrada, y construir matrices **diagonales** que normalicen cada término del costo a $[0, 1]$.

$$
Q = \mathrm{diag}\!\left(\frac{1}{x_{1,\max}^2},\,\frac{1}{x_{2,\max}^2},\,\ldots\right),
\qquad
R = \mathrm{diag}\!\left(\frac{1}{u_{1,\max}^2},\,\ldots\right)
$$

Para el motor BDC, valores razonables:

| Variable | Valor máximo "tolerable" | Peso |
|----------|--------------------------|------|
| $i_{a,\max}$ | 5 A | $1/25$ |
| $\omega_{\max}$ | 200 rad/s | $1/40\,000$ |
| $\theta_{\max}$ | $\sim 20°$ ($0{,}349$ rad) | $1/0{,}122$ |
| $u_{\max}$ | 24 V | $1/576$ |

A partir de esta línea base se afina iterativamente:

- **Acelerar el seguimiento de $\theta$:** multiplicar $Q_{33}$ por 10–100.
- **Disminuir el voltaje pedido:** multiplicar $R$ por 10–100.
- **Suavizar transitorios de corriente:** subir $Q_{11}$.

## 9.6 Seguimiento de Referencia

El LQR puro es un **regulador** ($r=0$). Para que el sistema siga una referencia $r\ne 0$ se aplican las mismas técnicas del cap. 07:

1. **Pre-compensación $K_{dc}$** (estática):

   $$ K_{dc} = \frac{1}{C_d\,(I - (\Phi - \Gamma K))^{-1}\,\Gamma},\qquad u = -K\,x + K_{dc}\,r. $$

   Simple, pero sensible a errores de modelo y a perturbaciones constantes.

2. **LQR con acción integral** (LQI): se aumenta el estado con $x_i[k+1] = x_i[k] + (r - y[k])$ y se aplica LQR al sistema aumentado de orden $n+1$. Garantiza error cero en estado estacionario y rechazo de perturbaciones.

3. **LQR servo** con ganancia precalculada para la referencia (state-feedforward).

En este capítulo usamos la opción 1 para alinearnos con la metodología del cap. 07 y poder comparar.

## 9.7 LQR vs. Pole-Placement

| Aspecto | Pole-placement (cap. 07) | LQR (cap. 09) |
|---------|--------------------------|----------------|
| Especificación | Polos deseados | Pesos $Q$, $R$ |
| Esfuerzo de control | No se considera explícitamente | Penalizado en $J$ |
| Garantía de estabilidad | Sí, si los polos están en $\|z\|<1$ | Sí, automática |
| Robustez | Variable | Margen de ganancia $[\tfrac{1}{2},\infty)$, margen de fase $\ge 60°$ (en continuo; en discreto se conservan propiedades similares con $T_s$ chico) |
| Escalabilidad MIMO | Requiere `place` cuidadoso | `dlqr` resuelve directamente |

## 9.8 Algoritmo en el Microcontrolador

```c
// Pre-cálculo offline en MATLAB:
//   [K, P, eig_cl] = dlqr(Phi, Gamma, Q, R);
//   Kdc = 1 / (Cd * inv(I - (Phi - Gamma*K)) * Gamma);
// Guardar K[3] y Kdc en flash.

void timer_isr(void) {            // cada Ts segundos
    leer_estados(x);              // medidos o estimados (cap. 08)
    float u = -K[0]*x[0] - K[1]*x[1] - K[2]*x[2] + Kdc * referencia;
    u = saturate(u, V_MIN, V_MAX);
    write_pwm(u);
}
```

El costo computacional en línea es **idéntico** al pole-placement: 3 multiplicaciones-acumulaciones por muestra. Toda la complejidad del LQR queda en el cálculo offline.

## 9.9 Validación

El script [lqr_bdc_z.m](../09_control_lqr/lqr_bdc_z.m) construye **cuatro** controladores sobre la misma planta:

| Etiqueta | Configuración | Qué demuestra |
|----------|---------------|---------------|
| **A — baseline** | $Q, R$ por regla de Bryson | Punto de partida razonable |
| **B — rápido** | $Q_{33}\times 100$ | Más prisa por $\theta$ → respuesta rápida, más voltaje |
| **C — suave** | $R \times 100$ | Penaliza el voltaje → respuesta lenta, menos saturación |
| **PP — pole-placement** | $K$ del cap. 07 | Referencia visual |

Para cada uno simula el lazo con **saturación de actuador** ($\pm 24\,$V) y grafica:

1. Posición angular vs. referencia.
2. Esfuerzo de control con bandas de saturación.
3. Costo acumulado $J(t)$ (evaluado con $Q,R$ baseline para una comparación justa).
4. Polos en el plano Z.

Por consola se reportan $M_p$, $t_p$, $|u|_{\max}$ y $J_\infty$ de cada estrategia.

## 9.10 Extensiones (no implementadas aquí)

- **LQG** (Linear Quadratic Gaussian): LQR + filtro de Kalman → óptimo con ruido. Combinación directa con el observador del cap. 08.
- **MPC** (Model Predictive Control): generaliza LQR sobre un horizonte finito y permite manejar saturaciones **dentro** del problema de optimización.
- **iLQR / DDP**: extensión a sistemas no lineales mediante linealización iterativa.

## 9.11 Material

- [lqr_bdc_z.m](../09_control_lqr/lqr_bdc_z.m)

## 9.12 Ejemplo numérico (sintonizado base Bryson)

Con los parámetros del motor real, $T_s = 10\,$ms y los topes de Bryson del script ($i_{a,\max}=5$ A, $\omega_{\max}=200$ rad/s, $\theta_{\max}=20° \approx 0{,}349$ rad, $u_{\max}=24$ V):

$$ Q = \mathrm{diag}\bigl(1/25,\; 1/40\,000,\; 1/0{,}122\bigr),\qquad R = 1/576. $$

`dlqr` arroja un $K$ con **tercera componente dominante** (la penalización relativa sobre $\theta$ es la mayor de las tres en $Q$):

$$ K_{\text{LQR}} \approx [\,k_1,\;\; k_2,\;\; k_3\,],\qquad |k_3| \gg |k_2| > |k_1|, $$

con autovalores en lazo cerrado todos dentro del círculo unitario (estabilidad garantizada por construcción). El script imprime los valores exactos por consola.

### Márgenes de robustez

Una propiedad clásica del LQR continuo es **margen de ganancia infinito superior y 1/2 inferior**, **margen de fase $\ge 60^\circ$**. En el caso **discreto** esto se relaja: ya no hay garantías universales tan limpias, pero en la práctica con $T_s$ suficientemente pequeño respecto a los polos de lazo cerrado, los márgenes obtenidos siguen siendo muy generosos comparados con un diseño equivalente por pole-placement — razón por la cual el LQR es preferido en aplicaciones críticas (aeroespacial, automóvil).

### Comparación cuantitativa con los cuatro ajustes del script

| Ajuste | $Q_{33}$ | $R$ | Comportamiento esperado |
|--------|----------|-----|-------------------------|
| Bryson | $1/0{,}122$ | $1/576$ | Equilibrio referencia. |
| Rápido ($Q_{33}\!\times\!100$) | $100/0{,}122$ | $1/576$ | Sube agresivamente, riesgo de saturar. |
| Suave ($R\!\times\!100$) | $1/0{,}122$ | $100/576$ | Voltaje muy bajo, tiempo de subida largo. |
| Equivalente PP | — | — | Coincide con el resultado del capítulo previo. |

El costo acumulado $J(t) = \sum_{k=0}^{N} x^T Q x + u^T R u$ que grafica el script confirma que **el sintonizado base de Bryson minimiza $J$** — los otros tres son peores en $J$ por construcción del problema, aunque puedan ser mejores en métricas no incluidas en $Q,R$ (sobreimpulso, slew rate, etc.).
