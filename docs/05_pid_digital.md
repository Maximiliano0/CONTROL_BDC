# 5. PID Digital (Dominio Z) — Control de Posición del Motor BDC (3×3)

> Aplicación: control de **posición angular** ($\theta$) del motor BDC con el modelo de estados **3×3** y los **parámetros de la planta real** ($R_a=11\,\Omega$, $L_a=0{,}008\,\text{H}$, $K_b=0{,}0014$, $J_e=7{,}56\times10^{-4}\,\text{kg·m}^2$, $B_e=10^{-5}\,\text{N·m·s}$).

## 5.1 Estructura del PID Continuo

$$ C(s) = K_p + \frac{K_i}{s} + \frac{K_d\, s}{1 + T_f\, s} $$

El término derivativo se acompaña de un **filtro causal** $1/(1+T_f s)$ porque un derivador puro es no realizable y amplifica ruido.

## 5.2 Discretización del PID

A partir de aproximaciones backward / Tustin obtenemos las tres ramas:

- **Proporcional:** $u_p[k] = K_p\, e[k]$
- **Integral (rectangular hacia atrás):** $u_i[k] = u_i[k-1] + K_i\, T_s\, e[k]$
- **Derivativa filtrada (backward Euler en el filtro):**

$$ u_d[k] = \frac{T_f}{T_f + T_s}\, u_d[k-1] + \frac{K_d}{T_f + T_s}\,(e[k] - e[k-1]) $$

- **Salida total:** $u[k] = u_p[k] + u_i[k] + u_d[k]$

Estas ecuaciones son el **algoritmo a implementar** en el microcontrolador. Sólo requieren memoria del paso anterior y suman/multiplican constantes.

## 5.3 Especificaciones → Frecuencia de Cruce y Margen de Fase

Dadas $M_p$ y $t_p$ deseados:

$$ \zeta = \frac{-\ln M_p}{\sqrt{\pi^2+\ln^2 M_p}},\quad \omega_n = \frac{\pi}{t_p\sqrt{1-\zeta^2}} $$

$$ \omega_c \approx \omega_n,\qquad \mathrm{PM} \approx \arctan\!\left(\frac{2\zeta}{\sqrt{\sqrt{1+4\zeta^4}-2\zeta^2}}\right) $$

`pidtune(sys_planta_z, 'PIDF', wc, opts)` ajusta el PID en el dominio z para alcanzar esa frecuencia de cruce y margen de fase, sintetizando $K_p, K_i, K_d, T_f$.

## 5.4 Selección de $T_s$

- $f_s$ entre 10 y 30 veces $f_n$ (frecuencia natural del lazo cerrado).
- En el script de la clase se trabaja con $T_s = 1\,\text{ms}$ ($f_s = 1\,\text{kHz}$), valor seguro frente a la dinámica del lazo elegida.

## 5.5 Validación Gráfica

El script [pid_bdc_z.m](../05_pid_digital/pid_bdc_z.m) genera:

1. Lugar de las raíces discreto (vista global y zoom a polos dominantes) con `zgrid`.
2. Bode de lazo abierto $L(z) = C(z)\,G(z)$ con `margin` para verificar PM y $\omega_c$.
3. Respuesta temporal: posición $\theta[k]$ vs. ideal continuo.
4. Esfuerzo de control $u[k]$ con `stairs` (ZOH).
5. Señal de error $e[k]$.
6. Mapa de polos/ceros del lazo cerrado.

## 5.6 Material

- [pid_bdc_z.m](../05_pid_digital/pid_bdc_z.m)
- [pid_bdc_z_sim.slx](../05_pid_digital/pid_bdc_z_sim.slx)
