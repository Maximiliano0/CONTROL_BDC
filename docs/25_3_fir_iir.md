# 25.3 Filtros FIR e IIR — Diseño y Análisis en el Plano Z

> **Aplicación al curso:** todo lazo de control digital se beneficia de un **filtrado previo** sobre las señales medidas. Aquí comparamos las dos familias clásicas de filtros digitales — **FIR** (no recursivos) e **IIR** (recursivos) — usando como banco de pruebas un seno de 100 Hz contaminado con ruido blanco, filtrado a una frecuencia de corte de 150 Hz con $f_s = 1\,\text{kHz}$.

---

## 25.3.1 Ecuación general

La ecuación en diferencias de un filtro digital lineal e invariante en el tiempo es:

$$ \sum_{k=0}^{N} a_k \cdot y[n-k] = \sum_{k=0}^{M} b_k \cdot x[n-k] $$

Su función de transferencia en el dominio Z (normalizada con $a_0 = 1$):

$$ H(z) = \frac{Y(z)}{X(z)} = \frac{\sum_{k=0}^{M} b_k \, z^{-k}}{1 + \sum_{k=1}^{N} a_k \, z^{-k}} $$

| Tipo | Condición | Polos | Estabilidad |
| ---- | --------- | ----- | ----------- |
| **FIR** | $a_k = 0$ para $k \ge 1$ | Todos en $z = 0$ | Siempre estable |
| **IIR** | $\exists\, a_k \ne 0$ con $k \ge 1$ | Donde caigan las raíces de $A(z)$ | Estable sólo si $|z_{\text{polo}}| < 1$ |

---

## 25.3.2 Filtro FIR (Hamming, $M = 11$)

- Script: [fir.py](../255_filtros_digitales/FIR_IIR_3_/fir.py)
- Diseño por **método de ventaneo**: se calcula un $\text{sinc}$ ideal truncado y se multiplica por una ventana Hamming para reducir el rizado de Gibbs.

```python
b = signal.firwin(num_taps=11, cutoff=150/(fs/2), window='hamming')
```

- **Plano Z:** todos los **ceros** dispersos sobre y alrededor del círculo unitario; los polos (no se grafican) están todos en el origen.
- **Respuesta en frecuencia:** banda de paso plana hasta ~150 Hz, atenuación monotónica más arriba.
- **Fase lineal exacta** → retardo de grupo constante $\tau = (M-1)/2 = 5$ muestras = 5 ms.
- **Implementación en µC:**

$$ y[n] = \sum_{k=0}^{10} b_k \cdot x[n-k] $$

Una multiplicación + acumulación (MAC) por tap, sobre un buffer circular de 11 muestras.

### Ventajas / Desventajas FIR
- ✅ Estabilidad absoluta.
- ✅ Fase lineal → no distorsiona la **forma** de las señales (clave para mediciones).
- ✅ Robusto a errores de cuantización de coeficientes.
- ❌ Requiere muchos taps para selectividad alta (decenas o cientos).
- ❌ Latencia: $\tau \cdot T_s$.

---

## 25.3.3 Filtro IIR (Butterworth orden 4)

- Script: [iir.py](../255_filtros_digitales/FIR_IIR_3_/iir.py)
- Diseño **Butterworth** pasa-bajos: respuesta de módulo **maximalmente plana** en la banda de paso.

```python
b, a = signal.butter(N=4, Wn=150/(fs/2), btype='low')
```

- **Plano Z:** 4 polos (rojos) dentro del círculo unitario + 4 ceros (azules) en $z=-1$ (el comportamiento pasa-bajos lleva todos los ceros al punto de Nyquist).
- **Respuesta en frecuencia:** banda de paso plana, transición más abrupta que el FIR de 11 taps a costa de:
  - Fase **no lineal** (distorsión de forma).
  - Solo 4 + 4 + 1 = **9 multiplicaciones por muestra** (vs. 11 del FIR para una selectividad muy superior).
- **Implementación en µC:**

$$ y[n] = \sum_{k=0}^{4} b_k \cdot x[n-k] \;-\; \sum_{k=1}^{4} a_k \cdot y[n-k] $$

Recomendable estructurarlo en **secciones bicuadráticas (SOS, Direct-Form II)** para evitar problemas numéricos con coeficientes muy pequeños o grandes.

### Ventajas / Desventajas IIR
- ✅ Mucho menos costo computacional para igual selectividad.
- ✅ Latencia mínima.
- ❌ **Riesgo de inestabilidad** si los polos caen fuera del círculo unitario (puede ocurrir por cuantización en aritmética de punto fijo).
- ❌ Fase no lineal → no apto cuando importa la **forma de onda** (p. ej. fasores).

---

## 25.3.4 Comparativa rápida

| Aspecto | FIR (11 taps Hamming) | IIR (Butterworth orden 4) |
| ------- | --------------------- | ------------------------- |
| Multiplicaciones por muestra | 11 | 9 (5 b + 4 a) |
| Selectividad (pendiente) | Suave | Abrupta |
| Fase | Lineal | No lineal |
| Estabilidad | Garantizada | Condicionada |
| Sensibilidad a cuantización | Baja | Alta (en punto fijo) |
| Memoria de estado | $M-1 = 10$ x's | 4 x's + 4 y's |

Como regla práctica:

- Si la prioridad es **fidelidad de forma de onda** (medición, instrumentación) → **FIR**.
- Si la prioridad es **rechazo agresivo con bajo costo** (filtrado de ruido en lazos rápidos) → **IIR Butterworth/Bessel SOS**.

---

## 25.3.5 Aplicación al motor BDC

En el contexto de los capítulos 05–09:

- **Filtro antialiasing analógico** (RC) antes del ADC → obligatorio.
- **Filtro digital de medición** post-ADC:
  - **IIR Butterworth orden 2/4** sobre $i_a[k]$ para limpiar conmutación del PWM (≥ 20 kHz) que se cuela como aliasing.
  - **FIR de 8 / 16 taps** sobre $\omega[k]$ derivado de un encoder cuando importa la fase (entrada del observador del cap. 08).
- El filtro debe diseñarse **considerando el lazo de control**: cualquier filtro agrega fase y resta margen al PID/LQR. Una buena práctica es ubicar la frecuencia de corte **al menos una década por encima** del ancho de banda del lazo cerrado.

---

## 25.3.6 Material

- [fir.py](../255_filtros_digitales/FIR_IIR_3_/fir.py)
- [iir.py](../255_filtros_digitales/FIR_IIR_3_/iir.py)
