# 2. Control en Espacio de Estados — Asignación de Polos (Continuo, 3×3 Posición)

> A partir de este capítulo el motor BDC se modela en **espacio de estados 3×3** y se controla **posición angular del eje** ($y = \theta$).

## 2.1 Objetivo

Diseñar una ley de control por realimentación de estados $u = -K \cdot x + K_{dc} \cdot r$ que ubique los polos del sistema 3×3 de **posición** del motor BDC en posiciones deseadas para cumplir especificaciones temporales (sobreimpulso $M_p$ y tiempo pico $t_p$).

## 2.2 Modelo de la Planta (3×3 Posición)

Con $x = [i_a,\,\omega,\,\theta]^T$:

$$ A = \begin{bmatrix}-R_a/L_a & -K_b/L_a & 0\\ K_b/J_e & -B_e/J_e & 0\\ 0 & 1 & 0\end{bmatrix}, \quad B = \begin{bmatrix}1/L_a\\ 0\\ 0\end{bmatrix}, \quad C = \begin{bmatrix}0 & 0 & 1\end{bmatrix}, \quad D = 0 $$

En este capítulo se usan los parámetros didácticos: $R_a=0.5\,\Omega$, $L_a=0.5\,\text{H}$, $K_b=0.01$, $J_e=0.01\,\text{kg·m}^2$, $B_e=0.1\,\text{N·m·s}$.

## 2.3 Mapeo Especificaciones → Polos

A partir de un comportamiento de 2do orden subamortiguado:

$$
\zeta = \frac{-\ln M_p}{\sqrt{\pi^2 + \ln^2 M_p}}, \qquad
\omega_n = \frac{\pi}{t_p \cdot \sqrt{1-\zeta^2}}
$$

$$ s_{1,2} = -\zeta\omega_n \pm j \cdot \omega_n\sqrt{1-\zeta^2} $$

El **tercer polo** (no dominante) se elige $s_3 = -10 \cdot \zeta\omega_n$ para no afectar la dinámica dominante.

## 2.4 Cálculo de la Ganancia $K$

Si el par $(A, B)$ es **controlable** ($\mathrm{rank}\,\mathcal{C} = n$), existe $K$ tal que los autovalores de $A - BK$ coinciden con los polos deseados. Se usa `place(A,B,P)`.

### Verificación de controlabilidad

$$ \mathcal{C} = [B\;\; AB\;\; A^2 B] \in \mathbb{R}^{3\times 3} $$

Para el modelo 3×3 del motor BDC, $\mathrm{rank}\,\mathcal{C} = 3$ ⇒ todo polo es asignable. Esto es consecuencia física de que el voltaje $V_a$ excita la corriente, que produce torque, que mueve la velocidad y por tanto la posición.

### Fórmula de Ackermann (alternativa cerrada)

Para $n=3$:

$$ K = [0\;0\;1] \cdot \mathcal{C}^{-1} \cdot \alpha_d(A),\qquad \alpha_d(A)=(A-p_1 I)(A-p_2 I)(A-p_3 I) $$

MATLAB usa internamente algoritmos más robustos numéricamente (basados en formas de Schur), pero Ackermann muestra que $K$ es una **expresión cerrada** en términos de los polos y de las matrices del modelo.

## 2.5 Pre-Compensación $K_{dc}$

Aunque la planta tiene un integrador natural en $\theta$, al cerrar el lazo con $u=-Kx$ ese integrador se mueve y se pierde el seguimiento perfecto. Se compensa con:

$$ K_{dc} = \frac{1}{\mathrm{dcgain}\!\left( C(sI-(A-BK))^{-1}B \right)} $$

de modo que para una referencia escalón $r=1$ se obtenga $\theta_\infty = 1$.

### Derivación por teorema del valor final

A lazo cerrado, $\dot{x} = (A-BK)x + B \cdot K_{dc} \cdot r$. En estado estacionario con $r$ constante:

$$ 0 = (A-BK) \cdot x_\infty + B \cdot K_{dc} \cdot r \;\Rightarrow\; x_\infty = -(A-BK)^{-1}B \cdot K_{dc} \cdot r $$

$$ y_\infty = C x_\infty = \underbrace{-C(A-BK)^{-1}B}_{=\,\mathrm{dcgain}} \cdot K_{dc} \cdot r $$

Forzando $y_\infty = r$ resulta $K_{dc} = 1/\mathrm{dcgain}$. **Limitación crítica:** este $K_{dc}$ se calcula con el modelo nominal; si los parámetros reales difieren (envejecimiento, temperatura, fricción variable) habrá error de seguimiento. La solución robusta es añadir **acción integral** (que el PID del cap. 05 incorpora de forma natural).

## 2.6 Ley de Control

$$ u(t) = -K \cdot x(t) + K_{dc} \cdot r(t) $$

## 2.7 Esfuerzo de Control

El script grafica $u(t)$ porque en la práctica el voltaje aplicado al motor está limitado (24 V típicos en la planta real). Si el diseño exige picos mayores, **el sistema saturará** y el comportamiento se aparta del lineal. Esto motiva los temas posteriores de PID con anti-windup (cap. 06).

## 2.8 Material

- [pp_control_src.m](../02_control_espacio_estados/pp_control_src.m)
- [pp_control_sim.slx](../02_control_espacio_estados/pp_control_sim.slx)

## 2.9 Ejemplo numérico (parámetros didácticos)

Con $M_p = 0.40$ y $t_p = 0.1\ \text{s}$:

$$ \zeta = \frac{-\ln 0.40}{\sqrt{\pi^2 + \ln^2 0.40}} \approx 0.2800,\qquad
\omega_n = \frac{\pi}{0.1 \cdot \sqrt{1-0.28^2}} \approx 32.74\,\text{rad/s} $$

Polos deseados:

$$ s_{1,2} = -9.17 \pm j \cdot 31.43,\qquad s_3 = -10 \cdot 9.17 = -91.7 $$

Matriz de estado en lazo abierto (sustituyendo $R_a=L_a=0.5$, $K_b=0.01$, $J_e=0.01$, $B_e=0.1$):

$$ A = \begin{bmatrix} -1 & -0.02 & 0 \\ 1 & -10 & 0 \\ 0 & 1 & 0 \end{bmatrix},\quad B = \begin{bmatrix} 2 \\ 0 \\ 0 \end{bmatrix} $$

Ejecutando `K = place(A, B, [s1 s2 s3])` se obtiene aproximadamente

$$ K \approx [\,50.9,\;\; 4.11 \times 10^3,\;\; 4.30 \times 10^4\,] $$

y $K_{dc} \approx 4.30 \times 10^4$ (coincide con $K_3$ porque la salida es directamente el estado 3).

### Lectura ingenieril del resultado

- $K_1$ pequeño: la corriente apenas necesita ser corregida (la planta eléctrica es ya estable y rápida).
- $K_2$ y $K_3$ grandes: la mayor parte de la ley de control se basa en velocidad y posición, lo cual es esperable cuando el objetivo es regular $\theta$.
- El pico de $u(t)$ resultante supera 100 V (ver gráfico) → con un puente H de $\pm 24\ \text{V}$ este diseño **saturaría**. Justamente esa es la motivación del capítulo 06.
