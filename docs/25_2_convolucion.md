# 25.2 Convolución Discreta — Filtros FIR Implementados Manualmente

> **Aplicación al curso:** la **convolución** es la operación elemental con la que se implementa cualquier **filtro FIR** dentro de un microcontrolador. Antes de usar `scipy.signal.lfilter` o sus equivalentes en C, hay que entender qué hace cada multiplicación y cada acumulación. Estos scripts construyen esa intuición paso a paso, comenzando con el filtro más simple posible: el **promedio móvil**.

---

## 25.2.1 Definición

La convolución discreta entre la señal $x[n]$ y la respuesta al impulso $h[n]$ (de longitud $M$) es:

$$ y[n] = (x \ast h)[n] = \sum_{k=0}^{M-1} h[k] \cdot x[n-k] $$

Cada muestra de salida requiere $M$ multiplicaciones y $M-1$ sumas (una operación **MAC** = *multiply-accumulate*).

En un µC con buffer circular de tamaño $M$, esta es la rutina central del ISR de muestreo.

---

## 25.2.2 Caso 1: convolución con `numpy` (referencia de validación)

- Script: [convolve.py](../255_filtros_digitales/Convolution_2_/convolve.py)
- Genera una señal compuesta (10 Hz + 150 Hz) y la filtra con un kernel **moving-average** de $M$ taps:

$$ h[k] = \frac{1}{M}, \quad 0 \le k < M $$

- El promedio móvil es el FIR pasa-bajos más simple. Su respuesta en frecuencia es un $\text{sinc}$ de longitud $M$, lo que atena los componentes rápidos pero introduce ondulaciones en la banda de paso.
- En este caso $N$ es **un múltiplo exacto del período** $T_o$ → la FFT no presenta leakage y se aprecia con claridad la atenuación del componente de alta frecuencia.

---

## 25.2.3 Caso 2: buffer de longitud fija (lo que realmente pasa en el µC)

- Script: [limited_buffer.py](../255_filtros_digitales/Convolution_2_/limited_buffer.py)
- Repite el ejercicio pero con **$N = 128$ fijo** (potencia de 2 → FFT eficiente).
- $N = 128$ NO contiene un número entero de períodos de 10 Hz → aparece **leakage** (cap. 25.1) que hay que distinguir del efecto del filtro.
- Es la situación realista: en un µC el buffer se elige por restricciones de **memoria** y **velocidad de FFT**, no para encajar con la frecuencia de la señal.

---

## 25.2.4 Caso 3: convolución implementada manualmente — `conv1d(x, h)`

- Script: [my_convolution.py](../255_filtros_digitales/Convolution_2_/my_convolution.py)
- Implementa la suma de convolución con un **doble for** explícito:

```python
def conv1d(x, h):
    N, M = len(x), len(h)
    y = np.zeros(N + M - 1)
    for n in range(N + M - 1):
        for k in range(M):
            if 0 <= n - k < N:
                y[n] += h[k] * x[n - k]
    return y
```

- Es exactamente el algoritmo que se traduce a C para correr en el µC, sustituyendo el `for` externo por la ejecución periódica del ISR y el `for` interno por una rutina MAC sobre un **buffer circular**.
- Resultado: idéntico (numéricamente) al de `np.convolve`. La diferencia es solo de **performance**, no de funcionalidad.

---

## 25.2.5 Caso 4: Zero-padding para FFT eficiente

- Script: [zero_pad.py](../255_filtros_digitales/Convolution_2_/zero_pad.py)
- Rellena el bloque con ceros hasta la **próxima potencia de 2** ($N_{fft} \ge N$).
- Beneficios:
  - La FFT acelera muchísimo (algoritmos radix-2).
  - Se obtiene una **interpolación espectral** (más bins, sin agregar información nueva): permite estimar mejor la posición del pico.
- Limitación: NO aumenta la resolución real (la cual está fijada por $\Delta f = 1/(N \cdot T_s)$).

Esta técnica es la base de la **convolución rápida vía FFT** ($y = \text{IFFT}(\text{FFT}(x) \cdot \text{FFT}(h))$), que rinde más que la convolución directa cuando $M$ es grande.

---

## 25.2.6 Aplicación al lazo de control digital

- **Pre-filtro de medición**: una convolución FIR de pocos taps (típicamente $M \in [4, 16]$) sobre $i_a[k]$ o $\omega[k]$ elimina ruido de conmutación (PWM) antes de entrar al observador (cap. 08) o al PID (cap. 05).
- **Costo**: $M$ multiplicaciones por muestra → cómodo aún en µC modestos a $f_s = 1\,\text{kHz}$.
- **Compromiso**: cada muestra de salida sufre un **retardo de grupo** $\tau = (M-1)/2 \cdot T_s$ que debe contabilizarse en el diseño del lazo (puede comerse margen de fase).

---

## 25.2.7 Material

- [convolve.py](../255_filtros_digitales/Convolution_2_/convolve.py)
- [limited_buffer.py](../255_filtros_digitales/Convolution_2_/limited_buffer.py)
- [my_convolution.py](../255_filtros_digitales/Convolution_2_/my_convolution.py)
- [zero_pad.py](../255_filtros_digitales/Convolution_2_/zero_pad.py)
