# =============================================================================
# Cap. 25.3 — FILTRO IIR (Infinite Impulse Response)
# -----------------------------------------------------------------------------
# Propósito  : Diseñar un filtro IIR pasa-bajos Butterworth de orden 4 y
#              aplicarlo a una sinusoide contaminada con ruido blanco.
#              Visualizar:
#                1) plano Z con CEROS (azul) Y POLOS (rojo); los polos
#                   deben estar DENTRO del círculo unitario para que el
#                   filtro sea ESTABLE.
#                2) respuesta en frecuencia |H(e^{jw})| en dB.
#                3) señal original vs. filtrada.
#                4) espectros |X(f)| y |Y(f)|.
# Características IIR:
#              + Mucho menor orden que un FIR equivalente (menos MAC/m).
#              + Selectividad muy alta con pocos coeficientes.
#              - Fase no lineal (distorsión de forma de onda).
#              - Riesgo de inestabilidad si los polos caen fuera de |z|=1.
# Implementación: y[n] = sum_{k} b[k] x[n-k] - sum_{k>=1} a[k] y[n-k]
#                 (estructura recursiva: usa las salidas pasadas).
# Tema       : DSP para filtrado de sensores en sistemas de control discreto.
# Doc        : ../../docs/25_3_fir_iir.md
# =============================================================================

import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
from scipy.fftpack import fft

# ---- Utilidad gráfica: plano Z con ceros, polos y la circunferencia |z|=1 ----
def plot_zplane(b, a, ax):
    zeros = np.roots(b)
    poles = np.roots(a)
    ax.scatter(np.real(zeros), np.imag(zeros), marker='o', color='blue', label='Zeros')
    ax.scatter(np.real(poles), np.imag(poles), marker='x', color='red', label='Poles')
    circle = plt.Circle((0, 0), 1, color='black', fill=False, linestyle='dashed')
    ax.add_patch(circle)
    ax.axhline(0, color='black', linewidth=0.7)
    ax.axvline(0, color='black', linewidth=0.7)
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_title("Z-Plane", fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.6)

# ---- Unidades y constantes ----
_seg = float(1)
_Hz = float(1/_seg)
_kHz = _Hz * (10**3)

# ---- Parámetros del sistema de muestreo ----
fs = 1 * _kHz # Sampling frequency
Ts = 1/fs # Sampling period

# ---- Parámetros de la señal de prueba ----
fo = 100 * _Hz # Signal frequency
A = 1 # Signal amplitude
N = 256 # Buffer length

# ---- Vector temporal y señal contaminada con ruido ----
t = np.arange(0, N*Ts, Ts)
x_t = A * np.sin(2 * np.pi * fo * t) + 0.5 * np.random.randn(len(t))

# ---- Diseño IIR Butterworth pasa-bajos de orden 4 ----
# Butterworth: respuesta de módulo MAXIMALÍSTICAMENTE PLANA en la banda de
# paso. Buen compromiso entre fase y selectividad. Orden 4 => 4 polos.
fc = 150 * _Hz  # Cutoff frequency
cutoff = fc / (fs / 2)  # Normalized cutoff frequency (respecto a Nyquist)
b, a = signal.butter(4, cutoff, btype='low', analog=False)

# Print IIR transfer function
print("IIR Filter Transfer Function:")
print(f"Numerator Coefficients (b): {b}")
print(f"Denominator Coefficients (a): {a}")
print(f"Filter Order: {len(a) - 1}")
print(f"Cutoff Frequency: {fc} Hz")
print(f"Sampling Frequency: {fs} Hz")

# ---- Aplicar el filtro (ecuación en diferencias recursiva, ver `lfilter`) ----
# En un µC: y[n] = (1/a[0]) * ( sum_k b[k]*x[n-k] - sum_{k>=1} a[k]*y[n-k] )
# Suele implementarse como secciones bicuadráticas (Direct-Form II SOS) para
# evitar problemas numéricos con coeficientes de alta resolución.
y_t = signal.lfilter(b, a, x_t)

# ---- Espectros de magnitud ----
X_f = np.abs(fft(x_t))[:N//2]
Y_f = np.abs(fft(y_t))[:N//2]
freqs = np.linspace(0, fs/2, N//2)

# Create subplots with the same size
fig, axes = plt.subplots(2, 2, figsize=(10, 8))

# Plot Z-Plane
plot_zplane(b, a, axes[0, 0])

# Plot Frequency Response
w, H = signal.freqz(b, a, worN=8000, fs=fs)
axes[0, 1].plot(w/_kHz, 20 * np.log10(abs(H)), label="Magnitude Response", color='blue', linewidth=2)
axes[0, 1].set_title("IIR Frequency Response", fontsize=14, fontweight='bold')
axes[0, 1].set_xlabel("Frequency (KHz)")
axes[0, 1].set_ylabel("Magnitude (dB)")
axes[0, 1].legend()
axes[0, 1].grid(True, linestyle='--', alpha=0.6)

# Plot Original and Filtered Signal
axes[1, 0].plot(t, x_t, label="Original Signal", alpha=0.5, color='gray')
axes[1, 0].plot(t, y_t, label="Filtered Signal", linewidth=2, color='blue')
axes[1, 0].set_title("Signal Filtering", fontsize=14, fontweight='bold')
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Amplitude")
axes[1, 0].legend()
axes[1, 0].grid(True, linestyle='--', alpha=0.6)

# Plot Spectrum
axes[1, 1].plot(freqs/_kHz, X_f, label="Original Spectrum", alpha=0.5, color='gray')
axes[1, 1].plot(freqs/_kHz, Y_f, label="Filtered Spectrum", linewidth=2, color='blue')
axes[1, 1].set_title("Signal Spectrum", fontsize=14, fontweight='bold')
axes[1, 1].set_xlabel("Frequency (KHz)")
axes[1, 1].set_ylabel("Magnitude")
axes[1, 1].legend()
axes[1, 1].grid(True, linestyle='--', alpha=0.6)

# Adjust layout and show plots
plt.tight_layout()
plt.show()