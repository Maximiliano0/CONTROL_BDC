# =============================================================================
# Cap. 25.1 — WINDOWING (2/3): VENTANA RECTANGULAR Y LEAKAGE
# -----------------------------------------------------------------------------
# Propósito  : Mostrar el efecto de TRUNCAR una señal con una ventana
#              rectangular de longitud N/2. Al multiplicar x[n] por w[n]
#              (= 1 dentro de la ventana, 0 fuera) en el TIEMPO, en
#              FRECUENCIA convolucionamos el espectro de la señal con el
#              espectro de la ventana (sinc para la ventana rectangular).
#              Aparecen lóbulos laterales = SPECTRAL LEAKAGE: una frecuencia
#              pura se "desparrama" sobre frecuencias vecinas.
# Aplicación : Cuando un sensor entrega muestras durante un tiempo finito
#              (lo único que un µC puede hacer), su FFT siempre ve una
#              ventana rectangular implícita; este script revela el costo.
# Doc        : ../../docs/25_1_windowing.md
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
To = 1/fo       # Período
A = 1           # Amplitud

# ---- Longitud del buffer (1 período completo) ----
N = To/Ts

# ---- Vector temporal y señal de prueba: suma de un seno y su 2do armónico ----
t = np.arange(0, N*Ts, Ts)
x_t = A * np.sin(2 * np.pi * fo * t) + np.sin(2 * np.pi * (2*fo) * t)

# ---- VENTANA RECTANGULAR de la MITAD del buffer ----
# w[n] = 1 para n in [0, N/2), 0 en el resto. Equivale a un "recorte abrupto".
# Su transformada es una sinc → lobulado lateral alto → mucho leakage.
WN1 = N/2
w = np.zeros(len(t))
w[:int(WN1)] = 1
x_wt = x_t * w   # multiplicación en tiempo = convolución en frecuencia

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
axes[2].set_title('Rectangular Window')
axes[2].grid()

plt.tight_layout()
plt.show()

# ---- Espectros (FFT) de cada señal ----
X_f = np.fft.fft(x_t)
abs_X_f = np.abs(X_f)
f = np.arange(0, fs/2, fs/N)

X_wf = np.fft.fft(x_wt)
abs_X_wf = np.abs(X_wf)
f = np.arange(0, fs/2, fs/N)

# Espectro de la ventana: aproxima a la función sinc (Dirichlet kernel).
W_f = np.fft.fft(w)
abs_W_f = np.abs(W_f)

# ---- Visualización espectral: comparar FFT original vs ventaneada ----
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
