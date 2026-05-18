# =============================================================================
# Cap. 25.2 — CONVOLUCIÓN (2/3): BUFFER LIMITADO Y ZERO-PADDING IMPLÍCITO
# -----------------------------------------------------------------------------
# Propósito  : Repetir el ejemplo de convolución con un BUFFER FIJO (N=128),
#              típico de aplicaciones embebidas donde el tamaño de buffer
#              está dictado por la memoria disponible (no por el período de
#              la señal). Como N ya no es múltiplo de fs/fo, aparece leakage
#              en el espectro y la respuesta del filtro se ve afectada por
#              los bordes del buffer.
# Diferencia vs. convolve.py:
#              - convolve.py: N se ajusta al período de la señal (sin leakage).
#              - limited_buffer.py: N es una potencia de 2 fija (caso µC).
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

# ---- Buffer FIJO (caso µC: memoria limitada) ----
# N=128 es potencia de 2 → FFT eficiente, pero NO contiene un número entero
# de períodos de fo → leakage espectral inevitable.
N = 128

# ---- Vector temporal y señal de prueba (igual a convolve.py) ----
t = np.arange(0, N * Ts, Ts)

# Input signal
x_t = A * np.sin(2 * np.pi * fo * t) + A * np.sin(2 * np.pi * (15*fo) * t)

# Impulse response (moving average filter)
kernel_size = int(N/10)
h_t = np.zeros(len(t))
h_t[:kernel_size] = 1 

# ---- Convolution ----
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
