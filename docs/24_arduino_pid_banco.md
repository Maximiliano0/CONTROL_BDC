# 24. Arduino PID - Protocolo de Pruebas de Banco

Este protocolo valida la implementacion de [24_arduino_example/arduino_pid/arduino_pid.ino](../24_arduino_example/arduino_pid/arduino_pid.ino) en hardware real, con criterios de aceptacion medibles.

## 24.1 Objetivo

Verificar que el lazo PID digital de posicion:

- siga referencias angulares sin inestabilidad,
- gestione saturacion del actuador de forma controlada,
- descargue el integrador con anti-windup ante saturacion,
- mantenga repetibilidad entre corridas.

## 24.2 Instrumentacion minima

- Motor BDC + puente H + encoder incremental.
- Arduino (o compatible) con serial a 115200.
- Fuente de 12 V (perfil fijo de firmware).
- Registro de telemetria CSV del monitor serial (columnas: t_ms, ref_deg, pos_rad, error_rad, u_total_v, u_p_v, u_i_v, u_d_v).

## 24.3 Pre-checklist de seguridad

1. Eje libre de obstaculos y sin topes mecanicos para el rango de prueba.
2. Polaridad de puente H confirmada (direccion positiva/negativa coherente).
3. Encoder con conteo estable al mover manualmente el eje.
4. En primer encendido, usar referencias pequenas (+/-10 deg) hasta confirmar signo de realimentacion.

## 24.4 Configuracion a auditar

Registrar antes de cada corrida:

- `TS_US` y `Ts`.
- `V_MAX` / `V_MIN`.
- `Kp`, `Ki`, `Kd`, `Tf`, `Kaw`.
- Perfil de firmware (fijo: hardware 12V, Ts=1 ms y Vsat=+/-12 V) y modo de `Kaw` (`CTRL_KAW_MODE`).
- Si se reinicia o no el controlador al cambiar referencia (`CTRL_RESET_ON_SETPOINT`).

## 24.5 Secuencia de pruebas

### Prueba A - Sanidad de lazo y signo

1. Enviar `START` con eje en cero mecanico.
2. Aplicar escalones: +10 deg, 0 deg, -10 deg, 0 deg (esperar asentamiento entre cambios).

Criterio de aceptacion:

- El sistema converge en el sentido correcto en los cuatro escalones.
- No hay crecimiento oscilatorio sostenido.

### Prueba B - Seguimiento de referencia nominal

1. Aplicar escalones: +30 deg, +60 deg, -30 deg.
2. Guardar telemetria CSV de cada corrida.

Criterio de aceptacion sugerido (ajustable al banco):

- Error en regimen permanente: |e_ss| <= 2 deg.
- Sobreimpulso: Mp <= 25 % para 30 deg y 60 deg.
- Tiempo de establecimiento (banda +/-5 %): t_s <= 3 s.

### Prueba C - Saturacion y anti-windup

1. Aplicar un escalon grande (por ejemplo +120 deg) para forzar saturacion.
2. Repetir con escalon de retorno a 0 deg.
3. Observar columnas `u_total_v` y `u_i_v`.

Criterio de aceptacion:

- `u_total_v` alcanza limites y vuelve a zona lineal sin quedarse pegado en saturacion.
- `u_i_v` no queda cargado permanentemente tras salir de saturacion.
- La recuperacion al volver a 0 deg no presenta cola excesiva (drift lento prolongado).

### Prueba D - Repetibilidad

1. Repetir Prueba B tres veces consecutivas con la misma configuracion.

Criterio de aceptacion:

- Variacion de t_s entre corridas <= 15 %.
- Variacion de Mp entre corridas <= 10 % absoluto.

## 24.6 Plantilla de reporte

- Fecha:
- Hardware:
- Fuente y tension:
- Configuracion de firmware:
- Resultado Prueba A: PASS/FAIL + notas
- Resultado Prueba B: PASS/FAIL + metricas
- Resultado Prueba C: PASS/FAIL + metricas
- Resultado Prueba D: PASS/FAIL + metricas
- Conclusion general: APROBADO / REQUIERE RETUNING

## 24.7 Acciones correctivas recomendadas

- Si hay oscilacion sostenida: reducir `Kp` o `Kd`, revisar signo de encoder.
- Si hay error estacionario alto: aumentar `Ki` gradualmente.
- Si hay windup visible: aumentar `Kaw` o ajustar modo `CTRL_KAW_MODE`.
- Si hay mucha zona muerta: bajar `UMBRAL_ARRANQUE_V` y/o `PWM_MIN_ARRANQUE`.
