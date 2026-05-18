# 25.1 Ventaneo (Windowing) — Preacondicionamiento Espectral de Señales de Sensor

> **Aplicación al curso:** antes de aplicar la **FFT** a una señal de sensor (corriente $i_a[k]$, velocidad $\omega[k]$, posición $\theta[k]$) en un microcontrolador, se trabaja siempre con un **buffer finito** de $N$ muestras. La forma en que se "recorta" la señal determina la calidad del espectro y, en consecuencia, la confiabilidad de cualquier estimador de frecuencia, monitor de vibración o detector de fallas alimentado por DSP.

---

## 25.1.1 Motivación: ¿por qué necesitamos ventanas?

Un microcontrolador adquiere muestras a tasa $f_s$ y procesa bloques de longitud $N$. Esto equivale a multiplicar la señal infinita $x[n]$ por una **ventana rectangular**:

$$ x_w[n] = x[n] \cdot w[n], \qquad w[n] = \begin{cases} 1 & 0 \le n < N \\ 0 & \text{en otro caso} \end{cases} $$

En el dominio de la frecuencia esto se traduce en una **convolución** con la transformada de la ventana:

$$ X_w(e^{j\omega}) = X(e^{j\omega}) \ast W(e^{j\omega}) $$

La ventana rectangular tiene una $W(e^{j\omega})$ tipo $\text{sinc}$ con **lóbulos laterales altos** (≈ −13 dB). Esto produce dos efectos indeseables:

1. **Leakage espectral:** energía de una tonal "se derrama" hacia frecuencias vecinas.
2. **Pérdida de resolución y enmascaramiento** de componentes débiles cercanas a otras fuertes.

---

## 25.1.2 Caso ideal: longitud que contiene un número entero de períodos

Si $N \cdot T_s = k \cdot T_o$ con $k \in \mathbb{N}$, las frecuencias presentes caen **exactamente** en los bins de la DFT y el leakage desaparece.

- Script de referencia: [signal_test.py](../255_filtros_digitales/Windowing_1_/signal_test.py)
- Genera un seno puro y muestra la FFT como un único bin limpio: el "caso ideal" de medición espectral.

---

## 25.1.3 Caso real: ventana rectangular que NO captura períodos enteros

Cuando $N$ no es múltiplo del período, la ventana rectangular dispersa la energía espectral.

- Script de referencia: [windowing.py](../255_filtros_digitales/Windowing_1_/windowing.py)
- Toma solo $N/2$ muestras del seno → la FFT presenta lóbulos laterales claramente visibles. Es la situación típica en un µC con buffer fijo y señal asíncrona.

---

## 25.1.4 Ventanas suaves: comparación

Las ventanas no-rectangulares **atenúan los bordes** del bloque y reducen el leakage a costa de **ensanchar el lóbulo principal** (peor resolución).

| Ventana      | Atenuación lóbulo lateral | Ancho lóbulo principal | Uso típico |
| ------------ | ------------------------: | ---------------------: | ---------- |
| Rectangular  | −13 dB | $2 \cdot \frac{2\pi}{N}$ | Sólo si $N$ captura períodos enteros |
| Bartlett     | −25 dB | $4 \cdot \frac{2\pi}{N}$ | Compromiso simple, triangular |
| Hanning      | −31 dB | $4 \cdot \frac{2\pi}{N}$ | Propósito general, suave en bordes |
| Hamming      | −41 dB | $4 \cdot \frac{2\pi}{N}$ | Mejor relación lóbulos para análisis |
| Blackman     | −58 dB | $6 \cdot \frac{2\pi}{N}$ | Máxima atenuación de leakage |

- Script de referencia: [different_windows.py](../255_filtros_digitales/Windowing_1_/different_windows.py)
- Compara las cinco ventanas sobre la misma señal y muestra su efecto en la FFT.

---

## 25.1.5 Aplicación al lazo de control digital

En el contexto del motor BDC (caps. 05–09):

- Para **monitoreo en línea** del consumo de corriente (detección de saturación, fricción anómala, atascos): usar **Hanning** o **Hamming** sobre buffers de $N = 128 / 256$ muestras a 1 kHz.
- Para **estimación de frecuencia mecánica** (vibración del eje, resonancias): usar **Blackman** si se busca aislar tonales débiles.
- La ventana se calcula **una sola vez** y se almacena en una tabla constante del µC; el costo en tiempo real es sólo $N$ multiplicaciones por bloque.

---

## 25.1.6 Material

- [signal_test.py](../255_filtros_digitales/Windowing_1_/signal_test.py)
- [windowing.py](../255_filtros_digitales/Windowing_1_/windowing.py)
- [different_windows.py](../255_filtros_digitales/Windowing_1_/different_windows.py)
