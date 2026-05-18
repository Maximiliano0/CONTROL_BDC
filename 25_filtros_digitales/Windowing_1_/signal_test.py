# =============================================================================
# Cap. 25.1 — WINDOWING (1/3): SEÑAL DE PRUEBA Y SU FFT
# -----------------------------------------------------------------------------
# Propósito  : Generar la señal de prueba más simple (un único seno) muestreada
#              a `fs` y graficar su espectro mediante la FFT. Sirve como
#              referencia "ideal" antes de introducir el concepto de ventana:
#              cuando `N` contiene un número ENTERO de períodos de la señal,
#              la FFT muestra una sola línea espectral en `fo` y los lóbulos
#              laterales son nulos (no hay leakage).
# Tema       : DSP para filtrado de sensores en sistemas de control discreto.
# Dependencias: numpy, matplotlib.
# Cómo correr: python signal_test.py
# Doc        : ../../docs/25_1_windowing.md
# =============================================================================

import numpy as np
import matplotlib.pyplot as plt

# ---- Unidades y constantes (para que el código se lea con unidades físicas) ----
_seg = float(1)
_mseg = _seg * (10**-3)
_Hz = float(1/_seg)
_kHz = _Hz * (10**3)

# ---- Parámetros del sistema de muestreo ----
fs = 1 * _kHz   # Frecuencia de muestreo (1 kHz) => Nyquist = 500 Hz
Ts = 1/fs       # Período de muestreo

# ---- Parámetros de la señal de prueba ----
fo = 10 * _Hz   # Frecuencia fundamental del seno
To = 1/fo       # Período de la señal
A = 1           # Amplitud

# ---- Longitud del buffer ----
# N elegido para capturar EXACTAMENTE 1 período (N = fs/fo = 100). Esto evita
# leakage espectral cuando se aplica la FFT con ventana rectangular implícita.
N = To/Ts

# ---- Vector temporal y señal x[n] ----
t = np.arange(0, N*Ts, Ts)
x_t = A * np.sin(2 * np.pi * fo * t)

# ---- Gráfica de la señal muestreada en el tiempo ----
plt.stem(t/_mseg, x_t)
plt.xlabel('Time [ms]')
plt.ylabel('Amplitude')
plt.title('Sampled Sinusoidal Signal')
plt.grid()
plt.show()

# ---- FFT (espectro de magnitud, solo semieje positivo) ----
# Nota: 1/N normaliza la magnitud; se grafican solo las primeras N/2 muestras
# (frecuencias positivas hasta Nyquist).
X_f = np.fft.fft(x_t)
abs_X_f = np.abs(X_f)
f = np.arange(0, fs/2, fs/N)

plt.stem(f/_kHz, (1/N) * abs_X_f[:int((N+1)/2)])
plt.xlabel('Frequency [kHz]')
plt.ylabel('Magnitude')
plt.title('FFT of the Signal')
plt.grid()
plt.show()