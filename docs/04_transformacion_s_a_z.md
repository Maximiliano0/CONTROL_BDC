# 4. Transformación de S a Z

## 4.1 ¿Por qué necesitamos discretizar?

El controlador se diseñará y/o implementará en un microcontrolador. Es necesario obtener una representación discreta $G(z)$ a partir del modelo continuo $G(s)$ que **conserve** las propiedades dinámicas relevantes (estabilidad, ganancia DC, ancho de banda).

## 4.2 Métodos clásicos

| Método | Mapeo s → z | Características |
|--------|-------------|-----------------|
| **ZOH** (Zero-Order Hold) | Equivalente exacto al DAC con retención de orden cero | Preserva respuesta al escalón; preferido para control |
| **Impulse Invariant** | Iguala respuesta al impulso muestreada | Puede generar alias si $f_s$ no es alta |
| **Tustin / Bilineal** | $s \approx \dfrac{2}{T_s}\,\dfrac{z-1}{z+1}$ | Preserva estabilidad; introduce *frequency warping* |
| **Forward Euler** | $s \approx \dfrac{z-1}{T_s}$ | Simple pero puede inestabilizar |
| **Backward Euler** | $s \approx \dfrac{z-1}{z\,T_s}$ | Más estable que Forward |

En MATLAB: `c2d(G_s, Ts, 'zoh' | 'tustin' | 'impulse' | 'foh' | ...)`.

## 4.3 Mapeo del Plano S al Plano Z

Para un polo $s = \sigma + j\omega$ con muestreo $T_s$:

$$ z = e^{s T_s} = e^{\sigma T_s} \big( \cos(\omega T_s) + j\,\sin(\omega T_s) \big) $$

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

$$ \omega_{\text{prewarp}} = \frac{2}{T_s}\,\tan\!\left(\frac{\omega_c T_s}{2}\right) $$

## 4.7 Material

- [S2Z_1_.m](../04_transformacion_s_a_z/S2Z_1_.m)
- [S2Z_2_.m](../04_transformacion_s_a_z/S2Z_2_.m)
- [S2Z_1_Sim.slx](../04_transformacion_s_a_z/S2Z_1_Sim.slx)
