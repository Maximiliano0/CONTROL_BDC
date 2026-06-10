# 24. Arduino Example - PID de Posicion para Motor DC

Este directorio contiene una implementacion embebida del controlador PID digital de posicion para motor DC con encoder incremental, ejecutada en Arduino (ejemplo: Arduino UNO) y puente H (L298N o equivalente).

Archivo principal:

- [arduino_pid/arduino_pid.ino](arduino_pid/arduino_pid.ino)

## Objetivo

Controlar posicion angular del eje en lazo cerrado usando:

- PID discreto con derivada filtrada,
- saturacion de actuador en +/-12 V,
- anti-windup por back-calculation,
- ajustes practicos para friccion/umbral del puente H.

## Ley de control (estado actual)

En cada periodo de muestreo Ts:

- Error: `e[k] = r[k] - y[k]`
- Proporcional: `u_p[k] = Kp * e[k]`
- Derivada filtrada:
  - `u_d[k] = (Tf/(Tf+Ts))*u_d[k-1] + (Kd/(Tf+Ts))*(e[k]-e[k-1])`
- Integral con anti-windup:
  - `u_i[k] = u_i[k-1] + Ts*(Ki*e[k] + Kaw*sat_error[k-1])`
- Control calculado: `u_calc = u_p + u_i + u_d`
- Saturacion: `u_sat = sat(u_calc, -12, +12)`
- Error de saturacion: `sat_error = u_sat - u_calc`

Adicionalmente, el firmware incluye compensaciones practicas del driver:

- compensacion de caida del L298N,
- umbral de arranque,
- PWM minimo de arranque,
- empuje minimo anti-stiction cuando el error supera un umbral.

## Parametros principales

En `arduino_pid.ino`:

- `Kp`, `Ki`, `Kd`, `Tf`: ganancias del PID discreto.
- `CTRL_TS_US`: periodo de muestreo en microsegundos (actual: 1000 us).
- `CTRL_V_MAX`: limite de voltaje de control (actual: 12 V).
- `CTRL_KAW_MODE`: modo de calculo de `Kaw`.
  - 0: `Kaw = Kp/10`
  - 1: `Kaw = sqrt(Ki*Kd)`
  - 2: `Kaw` fijo (`CTRL_KAW_FIXED`)

## Comandos Serial

Baudrate: `115200`

Comandos soportados:

- `START`: toma la posicion actual como cero y habilita control.
- `STOP`: deshabilita control y pone PWM en cero.
- `HELP`: muestra ayuda.
- `MOTOR <V>`: prueba en lazo abierto aplicando voltaje saturado (ejemplo: `MOTOR 6`).
- `<numero>`: referencia en grados (ejemplo: `90`).

## Telemetria

Cada segundo imprime panel de depuracion y linea CSV:

- Encabezado:
  - `CSV_HEADER,t_ms,ref_deg,pos_rad,error_rad,u_total_v,u_p_v,u_i_v,u_d_v`
- Datos:
  - `CSV,t_ms,ref_deg,pos_rad,error_rad,u_total_v,u_p_v,u_i_v,u_d_v`

Esta salida permite registrar pruebas y ajustar ganancias en banco.

## Flujo de uso recomendado

1. Energizar motor/driver y conectar serial.
2. En monitor serial, verificar mensajes de inicio y configuracion `[CFG]`.
3. Colocar eje en referencia mecanica y enviar `START`.
4. Enviar referencias en grados (`30`, `90`, `120`, etc.).
5. Usar `STOP` para deshabilitar control.

## Notas de puesta a punto

- Si no se mueve: revisar cableado, polaridad y encoder con `MOTOR 6` / `MOTOR -6`.
- Si oscila: bajar `Kp` o `Ki`, o subir ligeramente `Kd`.
- Si tarda en llegar: subir `Kp` o `Ki` gradualmente.
- Si hay windup evidente: ajustar `CTRL_KAW_MODE` y `Kaw`.

## Relacion con el repositorio

Este ejemplo corresponde al bloque de implementacion embebida de los capitulos de PID digital y anti-windup del proyecto CONTROL_BDC. Se mantiene trazabilidad con la formulacion documentada en:

- [../docs/05_pid_digital.md](../docs/05_pid_digital.md)
- [../docs/06_anti_windup.md](../docs/06_anti_windup.md)
