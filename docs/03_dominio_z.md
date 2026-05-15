# 3. Dominio Z, Muestreo y Estabilidad Discreta

## 3.1 Muestreo y Retención (Sample & Hold)

Un sistema de control digital reemplaza el bloque continuo por:

```
r(t) → [ADC]→ r[k] → [μC: G_c(z)] → u[k] → [DAC/ZOH] → u(t) → planta G(s) → y(t) → [ADC]→ y[k]
```

- **Período de muestreo:** $T_s = 1/f_s$.
- **Teorema de Nyquist–Shannon:** $f_s > 2 f_{\max}$ del contenido de la señal.
- **Regla de ingeniería de control:** $f_s \approx 10 \cdots 30 \times f_{\text{lazo cerrado}}$.
- **Retenedor de Orden Cero (ZOH):** mantiene la salida del DAC constante durante todo el período → introduce un retardo equivalente de $T_s/2$ y un cero adicional fuera del círculo unitario.

Material: [Sampling and Hold.pdf](../03_dominio_z/Sampling%20and%20Hold.pdf).

## 3.2 Transformada Z

$$ X(z) = \mathcal{Z}\{x[k]\} = \sum_{k=0}^{\infty} x[k]\, z^{-k} $$

Propiedades clave:

- **Desplazamiento atrás:** $\mathcal{Z}\{x[k-1]\} = z^{-1} X(z)$.
- **Mapeo s↔z (vía ZOH):** $z = e^{sT_s}$.

Repaso completo: [Repaso Z.pdf](../03_dominio_z/Repaso%20Z.pdf).

## 3.3 Estabilidad: Círculo Unitario

Un sistema discreto LTI es estable **sí y solo si** todos los polos cumplen $|z_i| < 1$.

| Región del plano s | Región mapeada en z |
|--------------------|---------------------|
| Semiplano izquierdo ($\sigma<0$) | Interior del círculo unitario ($\|z\|<1$) — ESTABLE |
| Eje imaginario ($\sigma=0$) | Círculo unitario ($\|z\|=1$) — Marginal |
| Semiplano derecho ($\sigma>0$) | Exterior del círculo ($\|z\|>1$) — INESTABLE |

Demostración en script: [Z_Stability.m](../03_dominio_z/Z_Stability.m). Se utiliza un sistema de **ejemplo genérico** $G(z)=(z-0{,}5)/(z^2-1{,}5z+0{,}7)$ con $f_s=100\,\text{Hz}$ para ilustrar polos, ceros y respuesta en frecuencia digital — no es la planta del motor.

## 3.4 Lazo Cerrado Digital

Con realimentación negativa unitaria:

$$ H(z) = \frac{G(z)}{1 + G(z)} $$

Cerrar el lazo **redibuja** los polos siguiendo el lugar geométrico de las raíces discreto (`rlocus` con `Ts ≠ 0`). En [Z_CloseLoop.m](../03_dominio_z/Z_CloseLoop.m) se compara la respuesta al escalón usando `stairs` (representación real de la salida del DAC) sobre el mismo $G(z)$ genérico.

## 3.5 Aliasing y Periodicidad del Espectro

La respuesta en frecuencia de un sistema discreto $H(e^{j\omega T_s})$ es **periódica** con período $f_s$. Por debajo de Nyquist ($f<f_s/2$) la información es única; cualquier componente $f > f_s/2$ se replica (alias) dentro de la banda base. Por eso se requiere **filtro antialiasing analógico** previo al ADC.

## 3.6 Material

- [Z_Stability.m](../03_dominio_z/Z_Stability.m)
- [Z_CloseLoop.m](../03_dominio_z/Z_CloseLoop.m)
- [Repaso Z.pdf](../03_dominio_z/Repaso%20Z.pdf)
- [Sampling and Hold.pdf](../03_dominio_z/Sampling%20and%20Hold.pdf)
