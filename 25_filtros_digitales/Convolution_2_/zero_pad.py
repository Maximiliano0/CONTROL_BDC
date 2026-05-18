# =============================================================================
# Cap. 25.2 — ANEXO: ZERO-PADDING Y RESOLUCIÓN ESPECTRAL
# -----------------------------------------------------------------------------
# Propósito  : Mostrar cómo el ZERO-PADDING (rellenar el buffer con ceros
#              hasta la siguiente potencia de 2) mejora la INTERPOLACIÓN
#              del espectro: más puntos en frecuencia, picos más suaves.
#              ¡OJO!: el zero-padding NO mejora la resolución REAL del
#              espectro (la cual está dada por la duración total de la
#              ventana), solo añade puntos de muestreo en el dominio
#              de la frecuencia (interpolación).
# Útil para  : FFT eficiente (potencias de 2) y visualización suave del
#              espectro de señales de sensores con buffer corto.
# Doc        : ../../docs/25_2_convolucion.md
# =============================================================================

import numpy as np 
import matplotlib.pyplot as plt

# ---- Unidades y constantes ----
_seg = float(1)
_mseg = _seg * (10**-3)
_Hz = float(1/_seg)
_kHz = _Hz * (10**3)

# ---- Parámetros del sistema de muestreo ----
fs = 1 * _kHz  # Sampling frequency (1 kHz)
Ts = 1/fs      # Sampling period

# ---- Parámetros de la señal de prueba ----
fo = 100 * _Hz  # Signal frequency (100 Hz)
To = 1/fo      # Signal period
A = 1           # Signal amplitude

# ---- Longitud del buffer original (1 período de fo) ----
N = int(To/Ts)

# ---- Vector temporal y señal ----
t = np.arange(0, N*Ts, Ts)
x_t = A * np.sin(2 * np.pi * fo * t)

# ---- ZERO-PADDING a la siguiente potencia de 2 ----
# Esto permite usar el algoritmo FFT eficiente (radix-2) y obtener un
# espectro más "liso" (interpolación en frecuencia, no nueva información).
N_padded = 2**int(np.ceil(np.log2(len(x_t))))  # siguiente potencia de 2
x_t_padded = np.pad(x_t, (0, N_padded - len(x_t)), 'constant')

# Nuevo vector temporal para la señal rellenada con ceros.
t_padded = np.linspace(0, N_padded/fs, N_padded)

# Plot the sampled signal
plt.figure(figsize=(10, 8))

# Plot original signal
plt.subplot(4, 1, 1)
plt.stem(t/_mseg, x_t)
plt.xlabel('Time [ms]')
plt.ylabel('Amplitude')
plt.title('Original Sampled Sinusoidal Signal')
plt.grid()

# Plot zero-padded signal
plt.subplot(4, 1, 2)
plt.stem(t_padded/_mseg, x_t_padded)
plt.xlabel('Time [ms]')
plt.ylabel('Amplitude')
plt.title('Zero-Padded Sinusoidal Signal')
plt.grid()

# FFT of the original signal (without zero padding)
X_f_original = np.fft.fft(x_t)
abs_X_f_original = np.abs(X_f_original)
f_original = np.arange(0, fs, fs/N)  # Frequency vector from 0 to fs

# Plot the FFT of the signal without zero padding
plt.subplot(4, 1, 3)
plt.stem(f_original[:N//2] / _kHz, (1/N) * abs_X_f_original[:N//2])
plt.xlabel('Frequency [kHz]')
plt.ylabel('Magnitude')
plt.title('FFT of the Signal (Without Zero Padding)')
plt.grid()

# FFT of the signal (with zero padding)
X_f_padded = np.fft.fft(x_t_padded)
abs_X_f_padded = np.abs(X_f_padded)
f_padded = np.arange(0, fs, fs/N_padded)  # Frequency vector for padded signal

# Plot the FFT of the signal with zero padding
plt.subplot(4, 1, 4)
plt.stem(f_padded[:N_padded//2] / _kHz, (1/N_padded) * abs_X_f_padded[:N_padded//2])
plt.xlabel('Frequency [kHz]')
plt.ylabel('Magnitude')
plt.title('FFT of the Signal (With Zero Padding)')
plt.grid()

# Show all plots
plt.tight_layout()
plt.show()
