# =============================================================================
# Cap. 25.2 — CONVOLUCIÓN (1/3): np.convolve CON KERNEL MOVING-AVERAGE
# -----------------------------------------------------------------------------
# Propósito  : Mostrar el ESQUEMA BÁSICO de filtrado por convolución discreta
#                   y[n] = sum_k h[k] * x[n-k]
#              usando un kernel rectangular (moving-average) de longitud
#              N/10. Se compara la señal de entrada (fundamental + armónico
#              alto) con la salida filtrada y se muestran sus espectros
#              |X(f)|, |H(f)|, |Y(f)|. Verifica gráficamente la propiedad
#                   Y(f) = X(f) * H(f)                (producto en frecuencia)
#              equivalente a la convolución en tiempo.
# Tema       : DSP para filtrado de sensores en sistemas de control discreto.
# Cómo correr: python convolve.py
# Doc        : ../../docs/25_2_convolucion.md
# =============================================================================

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

# ---- Unidades y constantes ----
_seg = float(1)
_mseg = _seg * (10**-3)
_Hz = float(1 / _seg)
_kHz = _Hz * (10**3)

# ---- Parámetros del sistema de muestreo ----
fs = 1 * _kHz  # Sampling frequency
Ts = 1 / fs    # Sampling period

# ---- Parámetros de la señal de prueba ----
fo = 10 * _Hz  # Signal frequency
To = 1 / fo    # Signal period
A = 1          # Signal amplitude

# ---- Longitud del buffer (1 período de fo) ----
N = int(To / Ts)

# ---- Vector temporal y señal de prueba ----
# Se mezcla la fundamental (fo) con un componente de alta frecuencia (15*fo).
# El kernel moving-average debe atenuar el componente rápido.
t = np.arange(0, N * Ts, Ts)

# Input signal
x_t = A * np.sin(2 * np.pi * fo * t) + A * np.sin(2 * np.pi * (15*fo) * t)

# ---- Respuesta al impulso h[n] del filtro moving-average ----
# Kernel rectangular de longitud N/10: promedio de las últimas N/10 muestras.
# A mayor `kernel_size`, más atenuación de alta frecuencia pero mayor retardo.
kernel_size = int(N / 10)
h_t = np.zeros(len(t))
h_t[:kernel_size] = 1

# ---- Convolución discreta y[n] = (x * h)[n] ----
# Se trunca el resultado a len(x_t) para mostrarlo con el mismo eje temporal.
# `mode='full'` retorna len(x)+len(h)-1 muestras; `mode='same'` retorna len(x).
y_t = np.convolve(x_t, h_t, mode='full')[:len(x_t)]
#y_t = np.convolve(x_t, h_t, mode='same')[:len(x_t)]
# Time vector for y_t
t_y = np.arange(0, len(y_t)) * Ts

# ---- SPECTRUM ----
X_f = np.fft.fft(x_t, N)
H_f = np.fft.fft(h_t, N)
Y_f = np.fft.fft(y_t, N)
freq = np.fft.fftfreq(N, Ts)

# ---- PLOTS ----
plt.figure(figsize=(14, 12))
gs = gridspec.GridSpec(3, 2)

# x(t)
plt.subplot(gs[0, 0])
plt.stem(t * 1e3, x_t, basefmt=" ", use_line_collection=True)
plt.title('Input Signal x(t)')
plt.xlabel('Time [ms]')
plt.ylabel('Amplitude')
plt.grid(True)

# h(t)
plt.subplot(gs[0, 1])
plt.stem(t * 1e3, h_t, basefmt=" ", use_line_collection=True)
plt.title('Kernel h(t)')
plt.xlabel('Time [ms]')
plt.ylabel('Amplitude')
plt.grid(True)

# |X(f)|
plt.subplot(gs[1, 0])
plt.stem(freq[:N // 2] / 1e3, np.abs(X_f[:N // 2]), basefmt=" ", use_line_collection=True)
plt.title('Spectrum |X(f)|')
plt.xlabel('Frequency [kHz]')
plt.ylabel('Magnitude')
plt.grid(True)

# |H(f)|
plt.subplot(gs[1, 1])
plt.stem(freq[:N // 2] / 1e3, np.abs(H_f[:N // 2]), basefmt=" ", use_line_collection=True)
plt.title('Spectrum |H(f)|')
plt.xlabel('Frequency [kHz]')
plt.ylabel('Magnitude')
plt.grid(True)

# y(t)
plt.subplot(gs[2, 0])
plt.stem(t_y * 1e3, y_t, basefmt=" ", use_line_collection=True)
plt.title('Output Signal y(t)')
plt.xlabel('Time [ms]')
plt.ylabel('Amplitude')
plt.grid(True)

# |Y(f)|
plt.subplot(gs[2, 1])
plt.stem(freq[:N // 2] / 1e3, np.abs(Y_f[:N // 2]), basefmt=" ", use_line_collection=True)
plt.title('Spectrum |Y(f)|')
plt.xlabel('Frequency [kHz]')
plt.ylabel('Magnitude')
plt.grid(True)

plt.tight_layout()
plt.show()
