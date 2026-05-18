# CONTROL_BDC

> Material didáctico del curso de **Control Discreto** (Ingeniería Electrónica) — usando un **Motor BDC (Brushed DC Motor)** como planta de referencia para integrar teoría y simulación.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b%2B-orange)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-R2024b%2B-blue)](https://www.mathworks.com/products/simulink.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Tabla de Contenidos

1. [Descripción](#descripción)
2. [Estructura del Repositorio](#estructura-del-repositorio)
3. [Hoja de Ruta del Curso](#hoja-de-ruta-del-curso)
4. [Modelo de la Planta](#modelo-de-la-planta)
5. [Requisitos](#requisitos)
6. [Cómo Ejecutar los Ejemplos](#cómo-ejecutar-los-ejemplos)
7. [Soporte Teórico](#soporte-teórico)
8. [Autor](#autor)
9. [Licencia](#licencia)

---

## Descripción

Este repositorio reúne todos los recursos utilizados para dictar el curso, organizados por bloque temático. Cada bloque combina:

- **Teoría:** archivo Markdown en [`docs/`](docs/) con desarrollo matemático en LaTeX.
- **Simulación:** scripts MATLAB (`.m`) y modelos Simulink (`.slx`).
- **Material complementario:** PDFs de apoyo.

El hilo conductor es el **diseño y validación de controladores digitales** (PID, asignación de polos en espacio de estados, anti-windup) sobre un motor BDC. **A partir de la sección 02 se utiliza el modelo de estados 3×3 del motor y se controla la posición angular del eje (θ).**

---

## Estructura del Repositorio

```text
CONTROL_BDC/
├── README.md                            ← este archivo
├── .gitignore
├── docs/                                ← apuntes teóricos en Markdown
│   ├── 00_indice.md
│   ├── 01_modelado_motor_bdc.md
│   ├── 02_control_espacio_estados.md
│   ├── 03_dominio_z.md
│   ├── 04_transformacion_s_a_z.md
│   ├── 05_pid_digital.md
│   ├── 06_anti_windup.md
│   ├── 07_control_estados_digital.md
│   ├── 08_observador_estados.md
│   └── 09_control_lqr.md
├── 01_modelado_motor_bdc/               ← Modelo del motor (2×2 velocidad)
│   ├── bdc_motor_src.m
│   ├── bdc_motor_sim.slx
│   └── Modelo de Motor BDC.pdf
├── 02_control_espacio_estados/          ← Asignación de polos (continuo, 3×3 posición)
│   ├── pp_control_src.m
│   └── pp_control_sim.slx
├── 03_dominio_z/                        ← Transformada Z, muestreo y estabilidad
│   ├── Z_Stability.m
│   ├── Z_CloseLoop.m
│   ├── Repaso Z.pdf
│   └── Sampling and Hold.pdf
├── 04_transformacion_s_a_z/             ← Discretización de plantas (S → Z)
│   ├── S2Z_1_.m
│   ├── S2Z_2_.m
│   └── S2Z_1_Sim.slx
├── 05_pid_digital/                      ← PID en dominio Z (3×3 posición)
│   ├── pid_bdc_z.m
│   └── pid_bdc_z_sim.slx
├── 06_anti_windup/                      ← Saturación + back-calculation (3×3 posición)
│   └── pid_windup_z.m
├── 07_control_estados_digital/          ← Asignación de polos en Z (3×3 posición)
│   └── pp_control_zrc.m
├── 08_observador_estados/               ← Observador Luenberger en Z (3×3 posición)
│   └── obs_control_z.m
└── 09_control_lqr/                      ← Control LQR discreto (3×3 posición)
    └── lqr_bdc_z.m
```

---

## Hoja de Ruta del Curso

| Clase | Tema | Carpeta | Apunte |
| ------: | ------ | --------- | -------- |
| 1 | Modelado del motor BDC (2×2 velocidad) | [`01_modelado_motor_bdc/`](01_modelado_motor_bdc/) | [docs/01](docs/01_modelado_motor_bdc.md) |
| 2 | Control por asignación de polos en continuo (3×3 posición) | [`02_control_espacio_estados/`](02_control_espacio_estados/) | [docs/02](docs/02_control_espacio_estados.md) |
| 3 | Repaso transformada Z, muestreo y estabilidad | [`03_dominio_z/`](03_dominio_z/) | [docs/03](docs/03_dominio_z.md) |
| 4 | Transformación S → Z (ZOH, Tustin, Impulse, etc.) | [`04_transformacion_s_a_z/`](04_transformacion_s_a_z/) | [docs/04](docs/04_transformacion_s_a_z.md) |
| 5 | PID digital sobre el motor BDC | [`05_pid_digital/`](05_pid_digital/) | [docs/05](docs/05_pid_digital.md) |
| 6 | Saturación del actuador y anti-windup | [`06_anti_windup/`](06_anti_windup/) | [docs/06](docs/06_anti_windup.md) |
| 7 | Asignación de polos en Z (control digital de estados) | [`07_control_estados_digital/`](07_control_estados_digital/) | [docs/07](docs/07_control_estados_digital.md) |
| 8 | Observador de estados (Luenberger) en Z | [`08_observador_estados/`](08_observador_estados/) | [docs/08](docs/08_observador_estados.md) |
| 9 | Control LQR discreto (cuadrático óptimo) | [`09_control_lqr/`](09_control_lqr/) | [docs/09](docs/09_control_lqr.md) |

---

## Modelo de la Planta

- **Sección 01 — Modelado introductorio:** modelo de **2×2** con estados $x = [i_a,\,\omega]^T$ (corriente y velocidad). Parámetros didácticos: $R_a = 0{,}5\,\Omega$, $L_a = 0{,}5\,\text{H}$, $K = 0{,}01$, $J_e = 0{,}01\,\text{kg·m}^2$, $B_e = 0{,}1\,\text{N·m·s}$.
- **Secciones 02 → 09:** se extiende a **3×3** agregando $x_3 = \theta$ con $\dot{x}_3 = \omega$. **La salida controlada es la posición angular del eje** ($C = [0\;0\;1]$).
- **Parámetros del motor real** usados en las secciones 05–09:

| Símbolo | Descripción | Valor |
| --------- | ------------- | ------- |
| $R_a$ | Resistencia de armadura | 11 Ω |
| $L_a$ | Inductancia de armadura | 0.008 H |
| $K_b = K$ | Constante FEM/torque | 0.0014 |
| $J_e$ | Inercia equivalente | 7.56 × 10⁻⁴ kg·m² |
| $B_e$ | Fricción viscosa | 1.0 × 10⁻⁵ N·m·s |
| $V_{\max}$ | Saturación del driver (puente H) | ±24 V |

---

## Requisitos

- **MATLAB** ≥ R2024b con los toolboxes:
  - Control System Toolbox
  - Signal Processing Toolbox
- **Simulink** ≥ R2024b para los modelos `*.slx`.

---

## Cómo Ejecutar los Ejemplos

```matlab
% Ejemplo: control PID discreto del motor BDC
cd('05_pid_digital')
pid_bdc_z
```

Cada script es **autónomo** (`clear; clc; close all` al inicio) y abre sus propias figuras. Los modelos Simulink se abren con doble clic o con `open_system('nombre.slx')`.

---

## Soporte Teórico

Toda la fundamentación matemática está en la carpeta [`docs/`](docs/), redactada en Markdown con ecuaciones LaTeX. El documento maestro es el **[índice](docs/00_indice.md)**.

Los apuntes están pensados para leerse antes de cada clase y volver durante el laboratorio para consultar fórmulas y procedimientos.

---

## Autor

**Ing. Maximiliano Vega**
Cátedra de Control Discreto — Ingeniería Electrónica

---

## Licencia

Este material se distribuye bajo licencia [MIT](LICENSE) para uso académico y profesional. Se agradece citar la fuente.
