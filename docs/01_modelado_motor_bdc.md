# 1. Modelado Matemático del Motor BDC

## 1.1 Introducción

El motor de corriente continua con escobillas (**Brushed DC Motor — BDC**) es la planta de referencia del curso. Su modelo electromecánico es lineal, de bajo orden y captura los fenómenos esenciales (resistencia, inductancia, FEM, inercia y fricción), lo que lo hace ideal para enseñar control clásico, moderno y digital.

## 1.2 Ecuaciones Físicas

### Subsistema Eléctrico (Ley de Kirchhoff de tensiones)

$$ V_a(t) = R_a\, i_a(t) + L_a\, \frac{d\,i_a(t)}{dt} + e_b(t) $$

donde la fuerza contraelectromotriz es proporcional a la velocidad angular:

$$ e_b(t) = K_b\, \omega(t) $$

### Subsistema Mecánico (Segunda ley de Newton para rotación)

$$ J_e\, \frac{d\,\omega(t)}{dt} = T_m(t) - B_e\, \omega(t) - T_L(t) $$

con torque electromagnético:

$$ T_m(t) = K\, i_a(t) $$

En unidades SI y motor ideal: $K_b = K$.

### Acople Posición — Velocidad

$$ \frac{d\,\theta(t)}{dt} = \omega(t) $$

## 1.3 Función de Transferencia (Velocidad)

Aplicando Laplace con condiciones iniciales nulas y $T_L=0$:

$$ G(s) = \frac{\Omega(s)}{V_a(s)} = \frac{K}{(J_e\,L_a)\,s^2 + (J_e R_a + B_e L_a)\,s + (B_e R_a + K^2)} $$

## 1.4 Espacio de Estados 2×2 — Velocidad (este capítulo)

Variables de estado: $x_1 = i_a$, $x_2 = \omega$.

$$
\dot{x}=
\begin{bmatrix}-R_a/L_a & -K/L_a \\ K/J_e & -B_e/J_e\end{bmatrix} x
+\begin{bmatrix}1/L_a \\ 0\end{bmatrix} V_a, \qquad
y = \begin{bmatrix}0 & 1\end{bmatrix} x
$$

> En el script `bdc_motor_src.m` se trabaja con esta representación 2×2 y se mide **velocidad angular $\omega$**.

## 1.5 Extensión a Espacio de Estados 3×3 — Posición (capítulos 02 → 07)

A partir del capítulo 02 se añade $x_3 = \theta$ con $\dot{x}_3 = x_2$ y se mide **posición angular**:

$$
A=\begin{bmatrix}-R_a/L_a & -K_b/L_a & 0\\ K_b/J_e & -B_e/J_e & 0\\ 0 & 1 & 0\end{bmatrix},
\;
B=\begin{bmatrix}1/L_a\\0\\0\end{bmatrix},
\;
C=\begin{bmatrix}0 & 0 & 1\end{bmatrix}
$$

> **Nota:** este modelo posee una **integradora pura** (de $\omega$ a $\theta$), por lo que el sistema es **Tipo 1** a lazo abierto.

## 1.6 Parámetros usados en el curso

| Símbolo | Descripción | Valor (clase intro — cap. 01 y 02) | Valor (planta real — cap. 05–07) |
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
