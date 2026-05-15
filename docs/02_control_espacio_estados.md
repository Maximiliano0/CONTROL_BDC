# 2. Control en Espacio de Estados — Asignación de Polos (Continuo, 3×3 Posición)

> A partir de este capítulo el motor BDC se modela en **espacio de estados 3×3** y se controla **posición angular del eje** ($y = \theta$).

## 2.1 Objetivo

Diseñar una ley de control por realimentación de estados $u = -K\,x + K_{dc}\, r$ que ubique los polos del sistema 3×3 de **posición** del motor BDC en posiciones deseadas para cumplir especificaciones temporales (sobreimpulso $M_p$ y tiempo pico $t_p$).

## 2.2 Modelo de la Planta (3×3 Posición)

Con $x = [i_a,\,\omega,\,\theta]^T$:

$$
A=\begin{bmatrix}-R_a/L_a & -K_b/L_a & 0\\ K_b/J_e & -B_e/J_e & 0\\ 0 & 1 & 0\end{bmatrix},
\;
B=\begin{bmatrix}1/L_a\\0\\0\end{bmatrix},
\;
C=\begin{bmatrix}0 & 0 & 1\end{bmatrix},
\; D = 0
$$

En este capítulo se usan los parámetros didácticos: $R_a=0{,}5\,\Omega$, $L_a=0{,}5\,\text{H}$, $K_b=0{,}01$, $J_e=0{,}01\,\text{kg·m}^2$, $B_e=0{,}1\,\text{N·m·s}$.

## 2.3 Mapeo Especificaciones → Polos

A partir de un comportamiento de 2do orden subamortiguado:

$$
\zeta = \frac{-\ln M_p}{\sqrt{\pi^2 + \ln^2 M_p}}, \qquad
\omega_n = \frac{\pi}{t_p\,\sqrt{1-\zeta^2}}
$$

$$ s_{1,2} = -\zeta\omega_n \pm j\,\omega_n\sqrt{1-\zeta^2} $$

El **tercer polo** (no dominante) se elige $s_3 = -10\,\zeta\omega_n$ para no afectar la dinámica dominante.

## 2.4 Cálculo de la Ganancia $K$

Si el par $(A, B)$ es **controlable** ($\mathrm{rank}\,\mathcal{C} = n$), existe $K$ tal que los autovalores de $A - BK$ coinciden con los polos deseados. Se usa `place(A,B,P)`.

## 2.5 Pre-Compensación $K_{dc}$

Aunque la planta tiene un integrador natural en $\theta$, al cerrar el lazo con $u=-Kx$ ese integrador se mueve y se pierde el seguimiento perfecto. Se compensa con:

$$ K_{dc} = \frac{1}{\mathrm{dcgain}\!\left( C(sI-(A-BK))^{-1}B \right)} $$

de modo que para una referencia escalón $r=1$ se obtenga $\theta_\infty = 1$.

## 2.6 Ley de Control

$$ u(t) = -K\,x(t) + K_{dc}\,r(t) $$

## 2.7 Esfuerzo de Control

El script grafica $u(t)$ porque en la práctica el voltaje aplicado al motor está limitado (24 V típicos en la planta real). Si el diseño exige picos mayores, **el sistema saturará** y el comportamiento se aparta del lineal. Esto motiva los temas posteriores de PID con anti-windup (cap. 06).

## 2.8 Material

- [pp_control_src.m](../02_control_espacio_estados/pp_control_src.m)
- [pp_control_sim.slx](../02_control_espacio_estados/pp_control_sim.slx)
