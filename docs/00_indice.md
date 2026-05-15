# Soporte Teórico — Índice

Curso: **Control Discreto** — Ingeniería Electrónica
Planta de referencia: **Motor BDC (Brushed DC Motor)**

> A partir de la sección **02** se utiliza el modelo de estados **3×3** del motor (con $\theta$ como tercer estado) y se controla **posición angular del eje**.

| # | Tema | Documento |
|---|------|-----------|
| 1 | Modelado matemático del motor BDC (modelo 2×2 — velocidad) | [01_modelado_motor_bdc.md](01_modelado_motor_bdc.md) |
| 2 | Control por asignación de polos en continuo (3×3 — posición) | [02_control_espacio_estados.md](02_control_espacio_estados.md) |
| 3 | Dominio Z, muestreo y estabilidad | [03_dominio_z.md](03_dominio_z.md) |
| 4 | Transformación de S a Z (ZOH, Tustin, Impulse, etc.) | [04_transformacion_s_a_z.md](04_transformacion_s_a_z.md) |
| 5 | PID digital (3×3 — posición) | [05_pid_digital.md](05_pid_digital.md) |
| 6 | Saturación del actuador y anti-windup (3×3 — posición) | [06_anti_windup.md](06_anti_windup.md) |
| 7 | Asignación de polos en Z — control digital de estados (3×3 — posición) | [07_control_estados_digital.md](07_control_estados_digital.md) |

Cada documento está acoplado al script y/o modelo Simulink correspondiente, ubicado en la carpeta hermana del repositorio raíz.
