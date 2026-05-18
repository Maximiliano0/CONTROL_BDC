# 3. Dominio Z, Muestreo y Estabilidad Discreta

## 3.1 Muestreo y Retención (Sample & Hold)

Un sistema de control digital reemplaza el bloque continuo por:

```text
r(t) → [ADC]→ r[k] → [μC: G_c(z)] → u[k] → [DAC/ZOH] → u(t) → planta G(s) → y(t) → [ADC]→ y[k]
```

- **Período de muestreo:** $T_s = 1/f_s$.
- **Teorema de Nyquist–Shannon:** $f_s > 2 f_{\max}$ del contenido de la señal.
- **Regla de ingeniería de control:** $f_s \approx 10 \cdots 30 \times f_{\text{lazo cerrado}}$.
- **Retenedor de Orden Cero (ZOH):** mantiene la salida del DAC constante durante todo el período → introduce un retardo equivalente de $T_s/2$ y un cero adicional fuera del círculo unitario.

Material: [Sampling and Hold.pdf](../03_dominio_z/Sampling%20and%20Hold.pdf).

## 3.2 Transformada Z

$$ X(z) = \mathcal{Z}\{x[k]\} = \sum_{k=0}^{\infty} x[k] \cdot z^{-k} $$

Propiedades clave:

- **Desplazamiento atrás:** $\mathcal{Z}\{x[k-1]\} = z^{-1} X(z)$.
- **Desplazamiento adelante** (con CI nulas): $\mathcal{Z}\{x[k+1]\} = z \cdot X(z)$.
- **Convolución discreta:** $\mathcal{Z}\{x*h\} = X(z)H(z)$.
- **Mapeo s↔z (vía ZOH):** $z = e^{sT_s}$.
- **Teorema del valor final:** $\displaystyle\lim_{k\to\infty} x[k] = \lim_{z\to 1}(1-z^{-1})X(z)$ si los polos están en $|z|<1$ excepto a lo sumo uno simple en $z=1$.

### Transformadas elementales

| Señal $x[k]$ ($k\ge 0$) | $X(z)$ | ROC |
|--------------------------|--------|-----|
| $\delta[k]$ | $1$ | todo $z$ |
| $u[k]$ (escalón) | $\dfrac{z}{z-1}$ | $\|z\|>1$ |
| $a^k \cdot u[k]$ | $\dfrac{z}{z-a}$ | $\|z\|>\|a\|$ |
| $k \cdot u[k]$ | $\dfrac{z}{(z-1)^2}$ | $\|z\|>1$ |
| $\sin(\omega_0 k) \cdot u[k]$ | $\dfrac{z\sin\omega_0}{z^2-2z\cos\omega_0+1}$ | $\|z\|>1$ |

Repaso completo: [Repaso Z.pdf](../03_dominio_z/Repaso%20Z.pdf).

## 3.3 Estabilidad: Círculo Unitario

Un sistema discreto LTI es estable **sí y solo si** todos los polos cumplen $|z_i| < 1$.

| Región del plano s | Región mapeada en z |
|--------------------|---------------------|
| Semiplano izquierdo ($\sigma<0$) | Interior del círculo unitario ($\|z\|<1$) — ESTABLE |
| Eje imaginario ($\sigma=0$) | Círculo unitario ($\|z\|=1$) — Marginal |
| Semiplano derecho ($\sigma>0$) | Exterior del círculo ($\|z\|>1$) — INESTABLE |

Demostración en script: [Z_Stability.m](../03_dominio_z/Z_Stability.m). Se utiliza un sistema de **ejemplo genérico** $G(z)=(z-0.5)/(z^2-1.5z+0.7)$ con $f_s=100\,\text{Hz}$ para ilustrar polos, ceros y respuesta en frecuencia digital — no es la planta del motor.

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

## 3.7 Equivalente ZOH: derivación formal

El ZOH mantiene $u(t) = u[k]$ constante durante $[kT_s,\,(k+1)T_s)$. Su transferencia equivalente en s es

$$ H_{\text{ZOH}}(s) = \frac{1 - e^{-sT_s}}{s}. $$

La cadena "ZOH + planta $G(s)$ + muestreador ideal" tiene equivalente discreto:

$$ \boxed{\;G_{\text{ZOH}}(z) = (1 - z^{-1}) \cdot \mathcal{Z}\!\left\{\frac{G(s)}{s}\right\}\;} $$

donde $\mathcal{Z}\{\cdot\}$ denota la transformación s→z por muestreo (`c2d` en MATLAB con `'zoh'`).

### Ejemplo numérico de mapeo s→z

Para $T_s = 10\ \text{ms}$ y un polo continuo $s = -5 + j10$:

$$ z = e^{(-5+j10)\cdot 0.01} = e^{-0.05}\bigl(\cos 0.1 + j\sin 0.1\bigr) \approx 0.9472 + j \cdot 0.0950 $$

$$ |z| \approx 0.9520 < 1 \;\Rightarrow\; \text{estable}. $$

Para que un polo continuo en $s = -100$ (rápido) se mapee a $z$ "saludablemente" lejos del origen y del círculo unitario hace falta $T_s$ del orden de $1/100 = 10\ \text{ms}$: con $T_s = 1\ \text{ms}$ se obtiene $z = e^{-0.1} \approx 0.9048$ (aún visible numéricamente), pero con $T_s = 0.01\ \text{ms}$ $z = e^{-0.001} \approx 0.9990$ → polos casi indistinguibles del origen → problemas numéricos al diseñar.

## 3.8 Estabilidad de Jury (criterio algebraico)

Análogo discreto del criterio de Routh–Hurwitz. Para $p(z) = a_n z^n + \cdots + a_0$, el sistema es estable sii:

1. $p(1) > 0$
2. $(-1)^n \cdot p(-1) > 0$
3. Una tabla de coeficientes (Jury) tiene todas las primeras columnas con el mismo signo.

Ejemplo: $p(z) = z^2 - 1.5 \cdot z + 0.7$ (denominador del ejemplo del capítulo).

- $p(1) = 1 - 1.5 + 0.7 = 0.2 > 0\;\checkmark$
- $p(-1) = 1 + 1.5 + 0.7 = 3.2 > 0\;\checkmark$
- $|a_0| = 0.7 < |a_2| = 1\;\checkmark$

→ sistema estable, consistente con que los polos $z_{1,2} = 0.75 \pm j \cdot 0.37$ tengan $|z| \approx 0.836 < 1$.
