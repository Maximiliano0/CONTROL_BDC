# 5. PID Digital (Dominio Z) — Control de Posición del Motor BDC (3×3)

> Aplicación: control de **posición angular** ($\theta$) del motor BDC con el modelo de estados **3×3** y los **parámetros de la planta real** ($R_a=11\,\Omega$, $L_a=0.008\,\text{H}$, $K_b=0.0014$, $J_e=7.56\times10^{-4}\,\text{kg·m}^2$, $B_e=10^{-5}\,\text{N·m·s}$).

## 5.1 Estructura del PID Continuo

$$ C(s) = K_p + \frac{K_i}{s} + \frac{K_d \, s}{1 + T_f \, s} $$

El término derivativo se acompaña de un **filtro causal** $1/(1+T_f s)$ porque un derivador puro es no realizable y amplifica ruido.

## 5.2 Discretización del PID

A partir de aproximaciones backward / Tustin obtenemos las tres ramas:

- **Proporcional:** $u_p[k] = K_p \cdot e[k]$
- **Integral (rectangular hacia atrás):** $u_i[k] = u_i[k-1] + K_i \cdot T_s \cdot e[k]$
- **Derivativa filtrada (backward Euler en el filtro):**

$$ u_d[k] = \frac{T_f}{T_f + T_s} \cdot u_d[k-1] + \frac{K_d}{T_f + T_s} \cdot (e[k] - e[k-1]) $$

- **Salida total:** $u[k] = u_p[k] + u_i[k] + u_d[k]$

Estas ecuaciones son el **algoritmo a implementar** en el microcontrolador. Sólo requieren memoria del paso anterior y suman/multiplican constantes.

### Derivación de la rama derivativa

Partimos del derivador con filtro causal $C_d(s) = \dfrac{K_d \, s}{1 + T_f \, s}$. Discretizando por backward Euler ($s \to (1 - z^{-1})/T_s$):

$$ C_d(z) = \frac{K_d \cdot (1 - z^{-1})/T_s}{1 + T_f \cdot (1 - z^{-1})/T_s} = \frac{K_d \cdot (1 - z^{-1})}{(T_f + T_s) - T_f \cdot z^{-1}}. $$

Reordenando $U_d(z) \bigl[(T_f + T_s) - T_f \cdot z^{-1}\bigr] = K_d \cdot (1 - z^{-1}) \cdot E(z)$ y antitransformando se llega exactamente a la recurrencia mostrada arriba.

### Forma posicional vs. incremental

La expresión anterior es **posicional** (calcula el valor absoluto de $u[k]$ cada vez). La **incremental** $\Delta u[k] = u[k] - u[k-1]$ es útil en sistemas con actuadores tipo "step" (válvulas), pero **no se recomienda con saturación y anti-windup** porque pierde la noción de la posición absoluta del integrador (esencial para el back-calculation del cap. 06).

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
2. Bode de lazo abierto $L(z) = C(z) \cdot G(z)$ con `margin` para verificar PM y $\omega_c$.
3. Respuesta temporal: posición $\theta[k]$ vs. ideal continuo.
4. Esfuerzo de control $u[k]$ con `stairs` (ZOH).
5. Señal de error $e[k]$.
6. Mapa de polos/ceros del lazo cerrado.

## 5.6 Material

- [pid_bdc_z.m](../05_pid_digital/pid_bdc_z.m)
- [pid_bdc_z_sim.slx](../05_pid_digital/pid_bdc_z_sim.slx)

## 5.7 Ejemplo numérico

Especificaciones del script: $M_p = 0.30$, $t_p = 100\ \text{s}$, $T_s = 1\ \text{ms}$.

$$ \zeta = 0.358,\quad \omega_n = 0.0336\,\text{rad/s},\quad \omega_c \approx 0.0336\,\text{rad/s},\quad \mathrm{PM} \approx 39.3^\circ $$

> El $t_p$ tan largo es didáctico: hace que la dinámica del cierre sea **mucho más lenta** que el polo eléctrico (que está en $\sim 1.4 \times 10^3$ rad/s) y que el modo mecánico real (cap. 01). Para una aplicación real ($t_p \sim 0.1\ \text{s}$) se obtendrían ganancias 1000× mayores y voltaje pico mucho más alto; ahí entraría la saturación del cap. 06.

`pidtune` devuelve típicamente coeficientes del orden de:

| Coef. | Valor aproximado | Comentario |
| ----- | ---------------- | ---------- |
| $K_p$ | $\sim 10^{-2}$ | dominado por la enorme constante mecánica efectiva |
| $K_i$ | $\sim 10^{-4}$ | acción integral muy lenta porque $t_p$ es grande |
| $K_d$ | $\sim 10^{-1}$ | predicción moderada |
| $T_f$ | $\sim 1$–10 s | filtro derivativo coherente con $\omega_c$ |

### Verificación del margen de fase

Ejecutando `[Gm,Pm,Wcg,Wcp] = margin(L_z)` sobre $L(z) = C(z) \cdot G(z)$ deberían obtenerse:

- $\omega_{cp} \approx \omega_c$ deseada,
- $\mathrm{PM} \approx 39^\circ$,
- $\mathrm{GM} \to \infty$ (planta sin cruce de fase relevante en el rango útil).

Si el PM medido difiere mucho del teórico, suele ser por una elección de $T_s$ demasiado próxima al ancho de banda objetivo, o por interacción con la dinámica eléctrica rápida.
