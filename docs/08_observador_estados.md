# 8. Observador de Estados (Luenberger) en Dominio Z

> Aplicación: control de **posición angular** del motor BDC con modelo **3×3** discretizado por ZOH y los **parámetros de la planta real**, asumiendo que **solo se mide la posición** ($\theta$) mediante un encoder.

## 8.1 Motivación

La ley de control por realimentación de estados del capítulo 07,

$$ u[k] = -K_z \cdot x[k] + K_{dc} \cdot r[k], $$

asume que **todos** los estados $x = [i_a,\,\omega,\,\theta]^T$ están disponibles. En la práctica:

- La **posición** $\theta$ se mide con un encoder incremental → barata y precisa.
- La **corriente** $i_a$ requiere un sensor de corriente (shunt + INA, sensor Hall) → mediciones ruidosas, costo extra.
- La **velocidad** $\omega$ habitualmente se **deriva** de $\theta$, lo que amplifica ruido a altas frecuencias.

La solución profesional consiste en **estimar** $i_a$ y $\omega$ en tiempo real con un **observador de estados**, usando solamente $\theta$ medida y el conocimiento del modelo de la planta. El controlador opera entonces sobre el **estado estimado** $\hat{x}[k]$.

## 8.2 Modelo de la Planta (recordatorio)

$$
x[k+1] = \Phi \cdot x[k] + \Gamma \cdot u[k],\qquad
y[k] = C_d \cdot x[k]
$$

con $\Phi = e^{A T_s}$, $\Gamma = \int_0^{T_s} e^{A\tau}d\tau \cdot B$, y $C_d = [0\;0\;1]$ (solo medimos $\theta$).

## 8.3 Observabilidad

Antes de diseñar un observador hay que verificar que el sistema sea **observable**: que las mediciones $y[k]$ contengan suficiente información para reconstruir $x[k]$.

$$
\mathcal{O} = \begin{bmatrix} C_d \\ C_d \cdot \Phi \\ C_d \cdot \Phi^2 \\ \vdots \\ C_d \cdot \Phi^{n-1} \end{bmatrix},\qquad \mathrm{rank}\,\mathcal{O} = n
$$

En MATLAB: `rank(obsv(Phi,Cd)) == n`. Para el motor BDC el par $(\Phi, C_d)$ con $C_d=[0\;0\;1]$ es observable, lo cual tiene sentido físico: la posición integra a la velocidad, que a su vez es función de la corriente; viendo $\theta$ a lo largo del tiempo y conociendo $u[k]$ se puede inferir el resto.

## 8.4 Estructura del Observador de Luenberger

Idea: copiar el modelo de la planta **dentro del microcontrolador** y corregirlo con la **innovación** $y[k] - C_d \cdot \hat{x}[k]$ (diferencia entre lo medido y lo que el modelo predice).

$$
\boxed{\;\hat{x}[k+1] = \Phi \cdot \hat{x}[k] + \Gamma \cdot u[k] + L \cdot \bigl(y[k] - C_d \cdot \hat{x}[k]\bigr)\;}
$$

Definiendo el **error de estimación** $e[k] = x[k] - \hat{x}[k]$:

$$
e[k+1] = (\Phi - L \cdot C_d) \cdot e[k]
$$

→ el error se rige **únicamente** por la dinámica $(\Phi - L \cdot C_d)$, **independiente** de $u[k]$ y $r[k]$. Si elegimos $L$ tal que los autovalores de $\Phi - L \cdot C_d$ estén estrictamente dentro del círculo unitario, $e[k]\to 0$ y $\hat{x}\to x$ asintóticamente.

### Derivación paso a paso de la dinámica del error

Restando observador menos planta:

$$
\begin{aligned}
x[k+1] - \hat{x}[k+1] &= \Phi \cdot x[k] + \Gamma \cdot u[k] \\
&\quad - \Phi \cdot \hat{x}[k] - \Gamma \cdot u[k] - L\bigl(y[k] - C_d\hat{x}[k]\bigr) \\
&= \Phi(x[k] - \hat{x}[k]) - L \cdot C_d\bigl(x[k] - \hat{x}[k]\bigr) \\
&= (\Phi - L C_d) \cdot e[k].
\end{aligned}
$$

**Observaciones clave:**

1. El control $u[k]$ se **cancela** porque entra idénticamente en planta y observador.
2. La referencia $r[k]$ no aparece (entra a la planta a través de $u$).
3. Las **perturbaciones no modeladas** en la planta sí afectan el error y no se cancelan; es el límite práctico del observador.

## 8.5 Diseño por Dualidad

El problema "ubicar polos de $\Phi - L \cdot C_d$" es **dual** a "ubicar polos de $\Phi - \Gamma K_z$":

$$
\mathrm{eig}(\Phi - L \cdot C_d) \;=\; \mathrm{eig}\bigl((\Phi - L \cdot C_d)^T\bigr) \;=\; \mathrm{eig}(\Phi^T - C_d^T \cdot L^T)
$$

Entonces se puede usar `place` sobre el sistema dual:

```matlab
L = place(Phi', Cd', P_obs).';
```

donde `P_obs` son los polos deseados del observador. **Regla práctica:** elegir los polos del observador entre **5 y 10 veces más rápidos** que los del controlador, para que la estimación converja antes de que afecte al control, sin amplificar excesivamente el ruido de medición.

## 8.6 Principio de Separación

Al cerrar el lazo $u[k] = -K_z \cdot \hat{x}[k] + K_{dc} \cdot r[k]$, el sistema completo en coordenadas $(x, e)$ queda:

$$
\begin{bmatrix} x[k+1] \\ e[k+1] \end{bmatrix} =
\underbrace{\begin{bmatrix} \Phi - \Gamma K_z & \Gamma K_z \\ 0 & \Phi - L \cdot C_d \end{bmatrix}}_{A_{\mathrm{aug}}}
\begin{bmatrix} x[k] \\ e[k] \end{bmatrix} +
\begin{bmatrix} \Gamma K_{dc} \\ 0 \end{bmatrix} r[k]
$$

La matriz es **triangular por bloques**, por lo que sus autovalores son la **unión** de los autovalores de $(\Phi - \Gamma K_z)$ y $(\Phi - L \cdot C_d)$. Conclusión:

> **El controlador y el observador pueden diseñarse por separado.**

Esto es el **Principio de Separación** (Separation Principle), uno de los resultados centrales del control moderno.

## 8.7 Variantes del Observador

| Tipo | Ecuación de actualización | Característica |
| ---- | ------------------------- | -------------- |
| **Predictor** (el implementado aquí) | $\hat{x}[k+1] = \Phi\hat{x}[k] + \Gamma u[k] + L(y[k] - C_d\hat{x}[k])$ | Usa $y[k]$ para predecir el estado **siguiente**. Introduce un retardo de 1 muestra. |
| **Current Estimator** | $\bar{x}[k] = \Phi\hat{x}[k{-}1] + \Gamma u[k{-}1]$, $\hat{x}[k] = \bar{x}[k] + L(y[k] - C_d\bar{x}[k])$ | Usa $y[k]$ para corregir el estado del **instante actual** antes de calcular $u[k]$. Mejor desempeño con $T_s$ grandes. |
| **Reducido** (Luenberger reducido) | Estima solo los estados **no medidos** | Menor orden ($n - p$) y menos cómputo, pero más sensible al ruido. |
| **Kalman discreto** | $L$ se calcula minimizando la varianza del error con modelos de ruido | Óptimo si los ruidos son gaussianos blancos. |

## 8.8 Algoritmo en el Microcontrolador

```c
// Pre-cálculo offline (PC/MATLAB) → tablas constantes en flash:
// Phi[3][3], Gamma[3], Cd[3], Kz[3], L[3], Kdc
// Estado persistente en RAM:
// float xhat[3] = {0, 0, 0};

void timer_isr(void) { // cada Ts segundos
 float y = read_encoder_position(); // medición
 float u = -(Kz[0]*xhat[0] + Kz[1]*xhat[1] + Kz[2]*xhat[2])
 + Kdc * reference;
 u = saturate(u, V_MIN, V_MAX); // saturación + (opcional) anti-windup
 write_pwm(u);

 // Innovación
 float innov = y - Cd[0]*xhat[0] - Cd[1]*xhat[1] - Cd[2]*xhat[2];

 // xhat[k+1] = Phi*xhat + Gamma*u + L*innov
 float xn[3];
 for (int i = 0; i < 3; i++) {
 xn[i] = Phi[i][0]*xhat[0] + Phi[i][1]*xhat[1] + Phi[i][2]*xhat[2]
 + Gamma[i]*u
 + L[i]*innov;
 }
 xhat[0] = xn[0]; xhat[1] = xn[1]; xhat[2] = xn[2];
}
```

Costo computacional: ~20 multiplicaciones-acumulaciones por iteración → trivial para cualquier MCU moderno.

## 8.9 Validación

El script [obs_control_z.m](../08_observador_estados/obs_control_z.m):

1. Calcula $K_z$ y $K_{dc}$ a partir de $(M_p, t_p)$ usando `place` (cap. 07).
2. Calcula $L$ con polos `factor_obs` veces más rápidos (por defecto **5×**).
3. **Verifica el principio de separación** comparando $\mathrm{eig}(A_{\mathrm{aug}})$ con $P_{\mathrm{ctrl}} \cup P_{\mathrm{obs}}$.
4. Simula **dos** sistemas en paralelo:
   - **Ideal:** $u = -K_z \cdot x$ (acceso al estado real, no realizable).
   - **Realista:** $u = -K_z \cdot \hat{x}$ con el observador corriendo en lazo.
5. Inicia el observador con un **error deliberado** en $\hat{\theta}$ para ver cómo converge.
6. Grafica posición, esfuerzo de control, corriente real vs. estimada, velocidad real vs. estimada, errores de estimación y polos en el plano Z.

## 8.10 Material

- [obs_control_z.m](../08_observador_estados/obs_control_z.m)

## 8.11 Ejemplo numérico

Mismas especificaciones que el cap. 07 ($M_p=0.10$, $t_p=1$, $T_s=10$ ms) con `factor_obs = 5`. Polos deseados del observador:

$$ s^{\text{obs}}_{1,2} = -11.51 \pm j \cdot 15.68,\qquad s^{\text{obs}}_3 = -115.1 $$

$$ z^{\text{obs}}_{1,2} = e^{-0.1151}\bigl(\cos 0.1568 \pm j\sin 0.1568\bigr) \approx 0.881 \pm j \cdot 0.139 $$

$$ z^{\text{obs}}_3 = e^{-1.151} \approx 0.316 $$

Mucho más próximos al origen que los polos del controlador ($\sim 0.98$) → el error de estimación decae en pocos pasos. La ganancia $L$ resultante es del orden de

$$ L \approx \begin{bmatrix} \sim 10^2 \\ \sim 10^1 \\ \sim 1 \end{bmatrix} $$

(los componentes asociados a estados no medidos son los más grandes, lo cual es consistente con la intuición: hay que "inventar" $i_a$ y $\omega$ a partir solo de $\theta$).

### Verificación del principio de separación

El script imprime `eig(Aaug)`; deben aparecer **6 autovalores** que coinciden (a menos de error numérico de `place`) con la unión de $P_{\text{ctrl}}$ (3 polos cerca de $z=1$) y $P_{\text{obs}}$ (3 polos cerca del origen). Si por error numérico se observara alguna desviación grande, podría deberse a que $P_{\text{ctrl}}$ y $P_{\text{obs}}$ son **demasiado próximos** entre sí (degeneración) — la regla "5×" precisamente busca evitarlo.

### Trade-off velocidad del observador vs. ruido

Subir `factor_obs` (polos más rápidos) acelera la convergencia pero **amplifica el ruido de medición**: $L$ actúa como un filtro pasaalto sobre $y[k]$. Una regla de oro: la varianza del estimado escala como $\sim \lVert L\rVert^2 \cdot \sigma_y^2$. Para encoders ópticos de buena resolución, `factor_obs = 5`–10 es seguro. Con sensores ruidosos conviene preferir el filtro de Kalman (que optimiza este trade-off automáticamente).
