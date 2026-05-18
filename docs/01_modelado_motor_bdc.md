# 1. Modelado Matemático del Motor BDC

## 1.1 Introducción

El motor de corriente continua con escobillas (**Brushed DC Motor — BDC**) es la planta de referencia del curso. Su modelo electromecánico es lineal, de bajo orden y captura los fenómenos esenciales (resistencia, inductancia, FEM, inercia y fricción), lo que lo hace ideal para enseñar control clásico, moderno y digital.

## 1.2 Ecuaciones Físicas

### Subsistema Eléctrico (Ley de Kirchhoff de tensiones)

$$ V_a(t) = R_a \cdot i_a(t) + L_a \cdot \frac{d\ i_a(t)}{dt} + e_b(t) $$

donde la fuerza contraelectromotriz es proporcional a la velocidad angular:

$$ e_b(t) = K_b \cdot \omega(t) $$

### Subsistema Mecánico (Segunda ley de Newton para rotación)

$$ J_e \cdot \frac{d\ \omega(t)}{dt} = T_m(t) - B_e \cdot \omega(t) - T_L(t) $$

con torque electromagnético:

$$ T_m(t) = K \cdot i_a(t) $$

En unidades SI y motor ideal: $K_b = K$.

### Acople Posición — Velocidad

$$ \frac{d\, \theta(t)}{dt} = \omega(t) $$

## 1.3 Función de Transferencia (Velocidad)

Aplicando Laplace con condiciones iniciales nulas y $T_L=0$:

$$ (R_a + L_a s) \cdot I_a(s) + K_b \cdot \Omega(s) = V_a(s) $$

$$ (J_e s + B_e) \cdot \Omega(s) = K \cdot I_a(s) $$

Despejando $I_a(s) = (J_e s + B_e) \cdot \Omega(s)/K$ y sustituyendo:

$$ \bigl[(R_a + L_a s)(J_e s + B_e) + K \cdot K_b\bigr] \cdot \Omega(s) = K \cdot V_a(s) $$

$$
\boxed{\; G(s) = \frac{\Omega(s)}{V_a(s)} = \frac{K}{(J_e L_a) \cdot s^2 + (J_e R_a + B_e L_a) \cdot s + (B_e R_a + K \cdot K_b)} \;}
$$

### Constantes de tiempo dominantes

Cuando $K \cdot K_b \ll B_e R_a$ (motores con poco acople electromecánico) la TF se factoriza aproximadamente como

$$ G(s) \approx \frac{K_{dc}}{(1 + \tau_e s)(1 + \tau_m s)},\quad
\tau_e = \frac{L_a}{R_a},\quad \tau_m = \frac{J_e}{B_e},\quad K_{dc} = \frac{K}{B_e R_a + K K_b}. $$

Típicamente $\tau_e \ll \tau_m$ → el modo eléctrico decae mucho más rápido y la respuesta queda gobernada por el modo mecánico.

## 1.4 Espacio de Estados 2×2 — Velocidad (este capítulo)

Variables de estado: $x_1 = i_a$, $x_2 = \omega$.

$$
\dot{x}=
\begin{bmatrix}-R_a/L_a & -K/L_a \\ K/J_e & -B_e/J_e\end{bmatrix} x
+\begin{bmatrix}1/L_a \\ 0\end{bmatrix} V_a, \qquad
y = \begin{bmatrix}0 & 1\end{bmatrix} x
$$

> En el script `bdc_motor_src.m` se trabaja con esta representación 2×2 y se mide **velocidad angular $\omega$**.

## 1.5 Extensión a Espacio de Estados 3×3 — Posición (capítulos 02 → 09)

A partir del capítulo 02 se añade $x_3 = \theta$ con $\dot{x}_3 = x_2$ y se mide **posición angular**:

$$ A = \begin{bmatrix}-R_a/L_a & -K_b/L_a & 0\\ K_b/J_e & -B_e/J_e & 0\\ 0 & 1 & 0\end{bmatrix}, \quad B = \begin{bmatrix}1/L_a\\ 0\\ 0\end{bmatrix}, \quad C = \begin{bmatrix}0 & 0 & 1\end{bmatrix} $$

> **Nota:** este modelo posee una **integradora pura** (de $\omega$ a $\theta$), por lo que el sistema es **Tipo 1** a lazo abierto.

## 1.6 Parámetros usados en el curso

| Símbolo | Descripción | Valor (clase intro — cap. 01 y 02) | Valor (planta real — cap. 05–09) |
|---------|-------------|------------------------------------|----------------------------------|
| $R_a$ | Resistencia de armadura | 0.5 Ω | 11 Ω |
| $L_a$ | Inductancia de armadura | 0.5 H | 0.008 H |
| $K_b = K$ | Constante de FEM/torque | 0.01 | 0.0014 |
| $J_e$ | Inercia equivalente | 0.01 kg·m² | 7.56 × 10⁻⁴ kg·m² |
| $B_e$ | Fricción viscosa | 0.1 N·m·s | 1.0 × 10⁻⁵ N·m·s |

## 1.7 Análisis (script `bdc_motor_src.m`)

El script reporta por consola la TF, las matrices A/B/C/D, polos y ceros; y grafica:

1. Respuesta al escalón con marcador de tiempo de asentamiento.
2. Mapa de polos y ceros.
3. Diagrama de Bode con márgenes de estabilidad.
4. Lugar geométrico de las raíces.
5. Evolución temporal de los estados $i_a(t)$ y $\omega(t)$.

## 1.8 Material

- [Modelo de Motor BDC.pdf](../01_modelado_motor_bdc/Modelo%20de%20Motor%20BDC.pdf)
- [bdc_motor_src.m](../01_modelado_motor_bdc/bdc_motor_src.m)
- [bdc_motor_sim.slx](../01_modelado_motor_bdc/bdc_motor_sim.slx)

## 1.9 Ejemplo numérico

### Parámetros didácticos (capítulos 01–02)

$R_a=0.5\,\Omega$, $L_a=0.5\,\text{H}$, $K_b=K=0.01$, $J_e=0.01\,\text{kg·m}^2$, $B_e=0.1\,\text{N·m·s}$.

| Magnitud | Expresión | Valor |
|----------|-----------|-------|
| $\tau_e$ | $L_a/R_a$ | $1.00\ \text{s}$ |
| $\tau_m$ | $J_e/B_e$ | $0.10\ \text{s}$ |
| $K_{dc}$ | $K/(B_e R_a + K K_b)$ | $\approx 0.1998\ \text{rad/(V·s)}$ |
| Polos de $G(s)$ | raíces de $0.005 \cdot s^2 + 0.055 \cdot s + 0.0501$ | $s_1 \approx -1.05$, $s_2 \approx -9.95$ |

Los polos están bien separados → la respuesta luce "de primer orden" en escala mecánica.

### Parámetros de planta real (capítulos 05–09)

$R_a = 11\,\Omega$, $L_a = 0.008\,\text{H}$, $K_b = K = 0.0014$, $J_e = 7.56 \times 10^{-4}\,\text{kg·m}^2$, $B_e = 10^{-5}\,\text{N·m·s}$.

| Magnitud | Expresión | Valor |
|----------|-----------|-------|
| $\tau_e$ | $L_a/R_a$ | $7.27 \times 10^{-4}\ \text{s}$ ($\approx 0.73\ \text{ms}$) |
| $\tau_m$ | $J_e/B_e$ | $75.6\ \text{s}$ |
| $K_{dc}$ | $K/(B_e R_a + K K_b)$ | $\approx 12.1\ \text{rad/(V·s)}$ |

La enorme separación $\tau_m/\tau_e \approx 10^5$ es típica de motores reales con baja inductancia y baja fricción: el modelo es **stiff** y exige $T_s \lesssim \tau_e/10 \approx 70\ \mu\text{s}$ si se quiere capturar la dinámica de corriente, o $T_s \approx 1$–10 ms si solo interesa la dinámica de posición (eligiendo $T_s = 1$ ms como en los capítulos 05–06).

### Verificación en MATLAB

```matlab
Ra=11; La=0.008; K=0.0014; Je=7.56e-4; Be=1e-5;
G = tf(K, [Je*La, Je*Ra+Be*La, Be*Ra+K*K]);
pole(G) % -> aprox [-1374.9, -1.46]
dcgain(G) % -> aprox 12.1
zpk(G)
```

Observar que el polo eléctrico ($\approx -1.375 \times 10^3$) corresponde a $1/\tau_e$ y el polo mecánico ($\approx -1.46$) a $1/\tau_m^{\text{eff}}$, donde $\tau_m^{\text{eff}}$ resulta menor que $\tau_m$ porque la fuerza contraelectromotriz añade amortiguamiento efectivo $K \cdot K_b/R_a$ a la dinámica mecánica.
