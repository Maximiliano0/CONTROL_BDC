# 7. Asignación de Polos en Dominio Z (Control de Estados Digital)

> Aplicación: control de **posición angular** del motor BDC con modelo **3×3** discretizado por ZOH y los **parámetros de la planta real**.

## 7.1 Objetivo

Implementar el control por realimentación de estados visto en el capítulo 02, pero **directamente discretizado** y listo para correr en un microcontrolador con un período $T_s$ definido por el diseñador.

## 7.2 Modelo Discreto de la Planta

$$ x[k+1] = \Phi \cdot x[k] + \Gamma \cdot u[k] $$

$$ y[k] = C_d \cdot x[k] $$

con $\Phi = e^{A T_s}$ y $\Gamma = \int_0^{T_s} e^{A\tau}\,d\tau \cdot B$ (ZOH exacto).

En MATLAB: `sys_d = c2d(sys_c, Ts, 'zoh')`.

### Fórmula cerrada cuando $A$ es invertible

Si $A$ es no singular,

$$ \int_0^{T_s} e^{A\tau}\,d\tau = A^{-1}\bigl(e^{A T_s} - I\bigr) \;\Rightarrow\; \Gamma = A^{-1}(\Phi - I) \cdot B. $$

Para el motor BDC 3×3 $A$ **es singular** (la fila de $\dot\theta$ es $[0\;1\;0]$, sin término propio), así que MATLAB usa una **expansión en serie** o la fórmula con matriz aumentada de Van Loan:

$$ \exp\!\left( T_s \begin{bmatrix} A & B \\ 0 & 0\end{bmatrix}\right) = \begin{bmatrix} \Phi & \Gamma \\ 0 & I \end{bmatrix}. $$

Este truco entrega $\Phi$ y $\Gamma$ simultáneamente con un solo `expm`.

## 7.3 Controlabilidad y Observabilidad Discretas

- **Controlabilidad:** $\mathcal{C} = [\Gamma\;\;\Phi\Gamma\;\;\Phi^2\Gamma\;\dots\;\Phi^{n-1}\Gamma]$, $\mathrm{rank}\,\mathcal{C}=n$.
- **Observabilidad:** $\mathcal{O} = [C_d;\;C_d\Phi;\;\dots;\;C_d\Phi^{n-1}]$, $\mathrm{rank}\,\mathcal{O}=n$.

Si la planta continua es controlable/observable, en general lo es la discretizada (excepto en los llamados *pathological sampling rates* donde $T_s$ coincide con periodos de modos oscilatorios).

## 7.4 Polos Deseados (Mapeo s → z)

A partir de las especificaciones $M_p,\,t_p$ se obtiene $\zeta,\omega_n$ y los polos continuos $s_1, s_2, s_3$. Luego:

$$ z_i = e^{s_i \cdot T_s},\qquad i=1,2,3 $$

## 7.5 Cálculo de la Ganancia $K_z$

$$ K_z = \mathrm{place}(\Phi, \Gamma, [z_1, z_2, z_3]) $$

de modo que los autovalores de $\Phi - \Gamma K_z$ sean los $z_i$.

## 7.6 Pre-Compensación $K_{dc}$

$$ K_{dc} = \frac{1}{C_d \cdot (I - (\Phi - \Gamma K_z))^{-1}\Gamma} $$

## 7.7 Algoritmo en el Microcontrolador

```c
// Cada Ts segundos (interrupción de timer):
y_meas = read_encoder_position(); // y[k]
estimar_estados(x, y_meas); // si no se miden todos: observador
u = -K_z[0]*x[0] - K_z[1]*x[1] - K_z[2]*x[2] + K_dc * referencia;
u = saturate(u, V_MIN, V_MAX);
write_pwm(u); // ZOH: queda hasta el próximo tick
```

## 7.8 Conversión de Unidades

El script [pp_control_zrc.m](../07_control_estados_digital/pp_control_zrc.m) acepta la referencia en **grados** (más natural para el ingeniero) y convierte internamente a **radianes** (necesarios para que la matemática del modelo en SI sea consistente).

## 7.9 Material

- [pp_control_zrc.m](../07_control_estados_digital/pp_control_zrc.m)

## 7.10 Ejemplo numérico

Con $M_p = 0.10$, $t_p = 1\ \text{s}$, $T_s = 10\ \text{ms}$ y los parámetros del motor real:

$$ \zeta = 0.591,\quad \omega_n = 3.90\,\text{rad/s},\quad \sigma = 2.30,\quad \omega_d = 3.14\,\text{rad/s} $$

Polos continuos deseados: $s_{1,2} = -2.30 \pm j \cdot 3.14$, $s_3 = -23.0$.

Mapeo $z_i = e^{s_i T_s}$ con $T_s = 0.01$:

$$ z_{1,2} = e^{-0.023}\bigl(\cos 0.0314 \pm j\sin 0.0314\bigr) \approx 0.9772 \pm j \cdot 0.0307 $$

$$ z_3 = e^{-0.230} \approx 0.7945 $$

Los tres polos están bien dentro del círculo unitario y cerca de $z=1$ (síntoma de dinámica lenta respecto a $T_s$, lo cual es deseable porque $T_s$ es claramente mucho más rápido que la dinámica del lazo).

La ganancia que devuelve `place` para este caso es del orden de:

$$ K_z \approx [\,0.06,\;\; 0.25,\;\; 6.8\,] $$

y $K_{dc}$ del orden de $6.8$ (cercana a $K_{z,3}$ porque la salida coincide con el tercer estado). El voltaje pico simulado queda dentro de los $\pm 24\ \text{V}$ → **no satura**, lo que valida la elección de $t_p = 1\ \text{s}$ para esta planta. Si se baja a $t_p = 0.1\ \text{s}$ las ganancias crecen $\sim 100\times$ y el voltaje pico requerido excede largamente la saturación.

## 7.11 Muestreo patológico

El par discreto $(\Phi, \Gamma)$ deja de ser controlable cuando $T_s$ coincide exactamente con un múltiplo del periodo de un modo oscilatorio. Para una planta con polos puramente imaginarios $\pm j\omega_0$, se pierde controlabilidad si $T_s = k\pi/\omega_0$. El motor BDC no presenta modos oscilatorios puros (su par de polos complejos siempre tiene parte real negativa), por lo que el problema no aparece en este curso, pero es importante recordarlo en sistemas resonantes (motores con flexibilidades, brazos robóticos largos, etc.).
