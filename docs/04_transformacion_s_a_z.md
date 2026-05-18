# 4. Transformación de S a Z

## 4.1 ¿Por qué necesitamos discretizar?

El controlador se diseñará y/o implementará en un microcontrolador. Es necesario obtener una representación discreta $G(z)$ a partir del modelo continuo $G(s)$ que **conserve** las propiedades dinámicas relevantes (estabilidad, ganancia DC, ancho de banda).

## 4.2 Métodos clásicos

| Método | Mapeo s → z | Características |
|--------|-------------|-----------------|
| **ZOH** (Zero-Order Hold) | Equivalente exacto al DAC con retención de orden cero | Preserva respuesta al escalón; preferido para control |
| **Impulse Invariant** | Iguala respuesta al impulso muestreada | Puede generar alias si $f_s$ no es alta |
| **Tustin / Bilineal** | $s \approx \dfrac{2}{T_s} \cdot \dfrac{z-1}{z+1}$ | Preserva estabilidad; introduce *frequency warping* |
| **Forward Euler** | $s \approx \dfrac{z-1}{T_s}$ | Simple pero puede inestabilizar |
| **Backward Euler** | $s \approx \dfrac{z-1}{z \cdot T_s}$ | Más estable que Forward |

En MATLAB: `c2d(G_s, Ts, 'zoh' | 'tustin' | 'impulse' | 'foh' | ...)`.

### Derivación breve de cada método

**Forward Euler:** se aproxima $\dot x(t) \approx \dfrac{x[k+1] - x[k]}{T_s}$. Tomando Laplace: $sX \to (z-1)/T_s \cdot X$.

*Mapa de estabilidad:* un polo continuo $s$ se mapea a $z = 1 + sT_s$. Para $s = -\sigma$ (polo estable), $z = 1 - \sigma T_s$; si $\sigma T_s > 2$, $|z| > 1$ → **inestable**. Esa es la principal trampa de Forward Euler para sistemas rápidos.

**Backward Euler:** $\dot x(t) \approx \dfrac{x[k] - x[k-1]}{T_s} \Rightarrow s \to (z-1)/(z T_s)$. El semiplano izquierdo se mapea **dentro** de un círculo de radio $1/2$ centrado en $1/2$: estable, pero distorsiona dinámicas rápidas.

**Tustin (bilineal):** integración trapezoidal. La aproximación $s \to \frac{2}{T_s}\frac{z-1}{z+1}$ es la transformación bilineal exacta entre los semiplanos: el eje $j\omega$ se mapea **biunívocamente** sobre el círculo unitario y el semiplano izquierdo sobre el interior. → **preserva estabilidad** siempre.

**ZOH:** discretización exacta del sistema lineal cuando la entrada es constante a trozos:

$$ x[k+1] = e^{A T_s} \cdot x[k] + \left(\int_0^{T_s} e^{A\tau}\,d\tau\right) B \cdot u[k]. $$

Para matrices invertibles, $\int_0^{T_s} e^{A\tau}d\tau = A^{-1}(e^{A T_s} - I)$. Es **exacto** entre instantes de muestreo (no aproximación) bajo la hipótesis de entrada con ZOH.

**Impulse Invariant:** iguala la respuesta al impulso muestreada. Adecuado para filtros de procesamiento de señales, no típico en control.

## 4.3 Mapeo del Plano S al Plano Z

Para un polo $s = \sigma + j\omega$ con muestreo $T_s$:

$$ z = e^{s T_s} = e^{\sigma T_s} \big( \cos(\omega T_s) + j \cdot \sin(\omega T_s) \big) $$

- **Constante $\sigma$ (decaimiento):** se mapea como circunferencias concéntricas en z. $\sigma<0$ → interior del círculo unitario.
- **Constante $\omega$ (frecuencia):** se mapea como rayos radiales. $\omega = \omega_s/2$ corresponde al punto $z=-1$.
- **Eje imaginario** ($\sigma=0$, $\omega$ variable): mapea sobre el **círculo unitario**.

Esta visualización aparece graficada en [S2Z_2_.m](../04_transformacion_s_a_z/S2Z_2_.m).

## 4.4 Efecto de $T_s$ en la posición de los polos

A medida que $T_s$ crece, los polos discretos se "desplazan" sobre el plano z y eventualmente cruzan el círculo unitario → **inestabilidad por sobre-muestreo lento**. Por eso $T_s$ se elige tal que los polos discretos queden bien dentro de $|z|<1$.

## 4.5 Comparación de Respuestas

El script [S2Z_1_.m](../04_transformacion_s_a_z/S2Z_1_.m) compara la respuesta al escalón y al impulso de un sistema **de ejemplo genérico** $G(s)=1/(2s^2+s+5)$ frente a $G_{\text{ZOH}}(z)$ y $G_{\text{Imp}}(z)$ con $f_s=1\,\text{kHz}$, para evidenciar las diferencias entre métodos.

## 4.6 Periodicidad y Frequency Warping

El método Tustin "comprime" el eje de frecuencias: un polo continuo a $\omega_c$ aparece en discreto a una frecuencia desplazada. Para diseños de filtros con corte preciso se usa **prewarping**:

$$ \omega_{\text{prewarp}} = \frac{2}{T_s} \cdot \tan\!\left(\frac{\omega_c T_s}{2}\right) $$

### Ejemplo numérico de warping

Filtro pasabajos continuo con $\omega_c = 2\pi \cdot 2 = 12.57\ \text{rad/s}$ y muestreo $f_s = 10\ \text{Hz}$ ($T_s = 100\ \text{ms}$):

$$ \omega_{c,\text{aparente}} = \frac{2}{T_s} \cdot \tan\!\left(\frac{\omega_c T_s}{2}\right) = 20 \cdot \tan(0.6283) \approx 14.53\,\text{rad/s} $$

Es decir, sin prewarping el filtro discretizado por Tustin **corta antes** (a $\omega \approx 12.57$) que el continuo (que cortaba a $\omega = 14.53$ del lado continuo equivalente). Para fijar el corte exacto a 2 Hz aplicamos prewarping invirtiendo la fórmula:

```matlab
opts = c2dOptions('Method','tustin','PrewarpFrequency', 2*pi*2);
LPF_z = c2d(LPF_s, Ts, opts);
```

## 4.7 Material

- [S2Z_1_.m](../04_transformacion_s_a_z/S2Z_1_.m)
- [S2Z_2_.m](../04_transformacion_s_a_z/S2Z_2_.m)
- [S2Z_1_Sim.slx](../04_transformacion_s_a_z/S2Z_1_Sim.slx)
