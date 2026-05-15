# 7. Asignación de Polos en Dominio Z (Control de Estados Digital)

> Aplicación: control de **posición angular** del motor BDC con modelo **3×3** discretizado por ZOH y los **parámetros de la planta real**.

## 7.1 Objetivo

Implementar el control por realimentación de estados visto en el capítulo 02, pero **directamente discretizado** y listo para correr en un microcontrolador con un período $T_s$ definido por el diseñador.

## 7.2 Modelo Discreto de la Planta

$$ x[k+1] = \Phi\, x[k] + \Gamma\, u[k] $$

$$ y[k] = C_d\, x[k] $$

con $\Phi = e^{A T_s}$ y $\Gamma = \int_0^{T_s} e^{A\tau}\,d\tau\,B$ (ZOH exacto).

En MATLAB: `sys_d = c2d(sys_c, Ts, 'zoh')`.

## 7.3 Controlabilidad y Observabilidad Discretas

- **Controlabilidad:** $\mathcal{C} = [\Gamma\;\;\Phi\Gamma\;\;\Phi^2\Gamma\;\dots\;\Phi^{n-1}\Gamma]$, $\mathrm{rank}\,\mathcal{C}=n$.
- **Observabilidad:** $\mathcal{O} = [C_d;\;C_d\Phi;\;\dots;\;C_d\Phi^{n-1}]$, $\mathrm{rank}\,\mathcal{O}=n$.

Si la planta continua es controlable/observable, en general lo es la discretizada (excepto en los llamados *pathological sampling rates* donde $T_s$ coincide con periodos de modos oscilatorios).

## 7.4 Polos Deseados (Mapeo s → z)

A partir de las especificaciones $M_p,\,t_p$ se obtiene $\zeta,\omega_n$ y los polos continuos $s_1, s_2, s_3$. Luego:

$$ z_i = e^{s_i\,T_s},\qquad i=1,2,3 $$

## 7.5 Cálculo de la Ganancia $K_z$

$$ K_z = \mathrm{place}(\Phi, \Gamma, [z_1, z_2, z_3]) $$

de modo que los autovalores de $\Phi - \Gamma K_z$ sean los $z_i$.

## 7.6 Pre-Compensación $K_{dc}$

$$ K_{dc} = \frac{1}{C_d\,(I - (\Phi - \Gamma K_z))^{-1}\Gamma} $$

## 7.7 Algoritmo en el Microcontrolador

```c
// Cada Ts segundos (interrupción de timer):
y_meas = read_encoder_position();        // y[k]
estimar_estados(x, y_meas);              // si no se miden todos: observador
u = -K_z[0]*x[0] - K_z[1]*x[1] - K_z[2]*x[2] + K_dc * referencia;
u = saturate(u, V_MIN, V_MAX);
write_pwm(u);                            // ZOH: queda hasta el próximo tick
```

## 7.8 Conversión de Unidades

El script [pp_control_zrc.m](../07_control_estados_digital/pp_control_zrc.m) acepta la referencia en **grados** (más natural para el ingeniero) y convierte internamente a **radianes** (necesarios para que la matemática del modelo en SI sea consistente).

## 7.9 Material

- [pp_control_zrc.m](../07_control_estados_digital/pp_control_zrc.m)
