# =============================================================================
# Cap. 25.1 — WINDOWING (3/3): COMPARACIÓN DE VENTANAS CLÁSICAS
# -----------------------------------------------------------------------------
# Propósito  : Comparar el efecto de DIFERENTES ventanas (Blackman, Hamming,
#              Hanning, Bartlett, Rectangular) sobre una señal politonal
#              (seno fundamental + 5to armónico). Cada ventana atena la
#              fuga espectral (leakage) en distinto grado, a costa de
#              ENSANCHAR el lóbulo principal.
# Compromiso clave:
#              - Rectangular  : lóbulo principal estrecho, laterales altos.
#              - Hanning/Hamming: balance típico (uso por defecto).
#              - Blackman     : lóbulos laterales muy bajos, principal ancho.
# Aplicación : Pre-procesamiento de señales de sensores antes de la FFT
#              para diagnóstico de vibración / ripple / armoniónicos.
# Doc        : ../../docs/25_1_windowing.md
#
# Para PROBAR cada ventana: descomentar el bloque correspondiente abajo.
# =============================================================================

import numpy as np
import matplotlib.pyplot as plt

# ---- Unidades y constantes ----
_seg = float(1)
_mseg = _seg * (10**-3)
_Hz = float(1/_seg)
_kHz = _Hz * (10**3)

# ---- Parámetros del sistema ----
fs = 1 * _kHz   # Frecuencia de muestreo
Ts = 1/fs       # Período de muestreo

# ---- Parámetros de la señal politonal de prueba ----
fo = 10 * _Hz   # Frecuencia fundamental
To = 1/fo       # Período de la fundamental
A = 1           # Amplitud

# ---- Longitud del buffer (1 período completo) ----
N = To/Ts

# ---- Vector temporal y señal de prueba ----
# Suma de la fundamental y del 5to armónico: útil para ver cómo cada
# ventana resuelve dos componentes espectrales cercanas.
t = np.arange(0, N*Ts, Ts)
x_t = A * np.sin(2 * np.pi * fo * t) + np.sin(2 * np.pi * (5*fo) * t)

# ---- Selección de la ventana ----
# Activa SOLO una a la vez. Las demás deben permanecer comentadas.
# Blackman: máxima atenuación de lóbulos laterales (~ -58 dB).
w = np.blackman(len(t))
x_wt = x_t * w
# Hamming: lóbulos laterales ~ -42 dB, lóbulo principal medio.
#w = np.hamming(len(t))
#x_wt = x_t * w
# Hanning: lóbulos ~ -31 dB, transición suave en los extremos.
#w = np.hanning(len(t))
#x_wt = x_t * w
# Bartlett (triangular): lóbulos ~ -26 dB, simple de calcular.
#w = np.bartlett(len(t))
#x_wt = x_t * w
# Rectangular (sin ventana real): máximo leakage, peor caso de referencia.
#w = np.ones(len(t))
#x_wt = x_t * w

# ---- Visualización temporal: señal, señal ventaneada y ventana ----
fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

axes[0].stem(t/_mseg, x_t)
axes[0].set_ylabel('Amplitude')
axes[0].set_title('Sampled Polytone Signal')
axes[0].grid()

axes[1].stem(t/_mseg, x_wt)
axes[1].set_ylabel('Amplitude')
axes[1].set_title('Windowed Polytone Signal')
axes[1].grid()

axes[2].stem(t/_mseg, w)
axes[2].set_xlabel('Time [ms]')
axes[2].set_ylabel('Amplitude')
axes[2].set_title('Window')
axes[2].grid()

plt.tight_layout()
plt.show()

# ---- Espectros (FFT) ----
# Comparamos la FFT de x[n], la de x[n]*w[n] y la de la propia w[n].
X_f = np.fft.fft(x_t)
abs_X_f = np.abs(X_f)
f = np.arange(0, fs/2, fs/N)

X_wf = np.fft.fft(x_wt)
abs_X_wf = np.abs(X_wf)
f = np.arange(0, fs/2, fs/N)

W_f = np.fft.fft(w)
abs_W_f = np.abs(W_f)

# ---- Visualización espectral ----
fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

axes[0].stem(f/_kHz, (1/N) * abs_X_f[:int((N+1)/2)])
axes[0].set_ylabel('Magnitude')
axes[0].set_title('FFT of the Signal')
axes[0].grid()

axes[1].stem(f/_kHz, (1/N) * abs_X_wf[:int((N+1)/2)])
axes[1].set_ylabel('Magnitude')
axes[1].set_title('FFT of the Windowed Polytone Signal')
axes[1].grid()

axes[2].stem(f/_kHz, (1/N) * abs_W_f[:int((N+1)/2)])
axes[2].set_xlabel('Frequency [kHz]')
axes[2].set_ylabel('Magnitude')
axes[2].set_title('FFT of the Window')
axes[2].grid()

plt.tight_layout()
plt.show()
