# =============================================================================
# Cap. 25.3 — FILTRO FIR (Finite Impulse Response)
# -----------------------------------------------------------------------------
# Propósito  : Diseñar un filtro FIR pasa-bajos por el método de VENTANEO
#              (firwin + Hamming) y aplicarlo a una sinusoide contaminada
#              con ruido blanco. Visualizar:
#                1) plano Z con los ceros del filtro (FIR → todos los polos
#                   en el origen z=0, son estables por construcción).
#                2) respuesta en frecuencia |H(e^{jw})| en dB.
#                3) señal original vs. señal filtrada en el tiempo.
#                4) espectros |X(f)| y |Y(f)|.
# Características FIR:
#              + Estabilidad garantizada (no hay polos fuera de z=0).
#              + Fase lineal exacta (retardo de grupo constante).
#              - Mayor orden (más taps) que un IIR para la misma selectividad.
# Implementación: y[n] = sum_{k=0..M-1} b[k] * x[n-k]   (convolución FIR)
# Tema       : DSP para filtrado de sensores en sistemas de control discreto.
# Doc        : ../../docs/25_3_fir_iir.md
# =============================================================================

import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
from scipy.fftpack import fft

# ---- Utilidad gráfica: plano Z con la circunferencia |z| = 1 ----
def plot_zplane(h, ax):
    zeros = np.roots(h)
    ax.scatter(np.real(zeros), np.imag(zeros), marker='o', color='blue', label='Zeros')
    circle = plt.Circle((0, 0), 1, color='black', fill=False, linestyle='dashed', linewidth=1.5)
    ax.add_patch(circle)
    ax.axhline(0, color='black', linewidth=0.75, linestyle='dashed')
    ax.axvline(0, color='black', linewidth=0.75, linestyle='dashed')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_title("Z-Plane", fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, linestyle='dotted')

# ---- Unidades y constantes ----
_seg = float(1)
_Hz = float(1/_seg)
_kHz = _Hz * (10**3)

# ---- Parámetros del sistema de muestreo ----
fs = 1 * _kHz  # Sampling frequency
Ts = 1/fs  # Sampling period

# ---- Parámetros de la señal de prueba ----
# Se contamina un seno de 100 Hz con ruido blanco (std=0.5) para evaluar
# cómo el filtro atena el ruido por encima de la frecuencia de corte.
fo = 100 * _Hz  # Signal frequency
A = 1  # Signal amplitude

# ---- Longitud del buffer ----
N = 256

# ---- Vector temporal y señal de prueba ----
t = np.arange(0, N*Ts, Ts)
x_t = A * np.sin(2 * np.pi * fo * t) + 0.5 * np.random.randn(len(t))

# ---- Diseño del FIR pasa-bajos (método de ventaneo, Hamming) ----
# num_taps : orden + 1. Más taps => transición más abrupta pero más cómputo.
# cutoff   : frecuencia de corte NORMALIZADA respecto a Nyquist (fs/2).
num_taps = 11
fc = 150 * _Hz  # Cutoff frequency
cutoff = fc / (fs / 2)  # Normalized cutoff frequency
b = signal.firwin(num_taps, cutoff=cutoff, window='hamming', fs=fs)

# Print FIR transfer function
print("FIR Filter Transfer Function:")
print(f"Numerator Coefficients (b): {b}")
print(f"Filter Order: {num_taps - 1}")
print(f"Cutoff Frequency: {fc} Hz")
print(f"Sampling Frequency: {fs} Hz")

# ---- Aplicar el filtro a la señal (convolución implementada por lfilter) ----
# En un µC esto se programa como un buffer circular de M=num_taps muestras
# y un acumulador MAC: y[n] = sum_k b[k] * x[n-k].
y_t = signal.lfilter(b, 1, x_t)

# ---- Espectros de magnitud (FFT, semieje positivo) ----
X_f = np.abs(fft(x_t))[:N//2]
Y_f = np.abs(fft(y_t))[:N//2]
freqs = np.linspace(0, fs/2, N//2)

# Create subplots with the same size
fig, axes = plt.subplots(2, 2, figsize=(10, 8))

# Plot Z-Plane
plot_zplane(b, axes[0, 0])

# Plot Frequency Response
w, h = signal.freqz(b, worN=8000, fs=fs)
axes[0, 1].plot(w/_kHz, 20 * np.log10(abs(h)), color='red', linewidth=2)
axes[0, 1].set_title("FIR Frequency Response", fontsize=14, fontweight='bold')
axes[0, 1].set_xlabel("Frequency (KHz)", fontsize=12)
axes[0, 1].set_ylabel("Magnitude (dB)", fontsize=12)
axes[0, 1].grid(True, linestyle='dotted')

# Plot Original and Filtered Signal
axes[1, 0].plot(t, x_t, label="Original Signal", alpha=0.5, color='gray')
axes[1, 0].plot(t, y_t, label="Filtered Signal", linewidth=2, color='blue')
axes[1, 0].set_title("Signal Filtering", fontsize=14, fontweight='bold')
axes[1, 0].set_xlabel("Time (s)", fontsize=12)
axes[1, 0].set_ylabel("Amplitude", fontsize=12)
axes[1, 0].legend()
axes[1, 0].grid(True, linestyle='dotted')

# Plot Spectrum
axes[1, 1].plot(freqs/_kHz, X_f, label="Original Spectrum", alpha=0.5, color='gray')
axes[1, 1].plot(freqs/_kHz, Y_f, label="Filtered Spectrum", linewidth=2, color='blue')
axes[1, 1].set_title("Signal Spectrum", fontsize=14, fontweight='bold')
axes[1, 1].set_xlabel("Frequency (KHz)", fontsize=12)
axes[1, 1].set_ylabel("Magnitude", fontsize=12)
axes[1, 1].legend()
axes[1, 1].grid(True, linestyle='dotted')

# Adjust layout and show plots
plt.tight_layout()
plt.show()
