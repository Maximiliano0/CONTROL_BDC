% =========================================================================
% Cap. 05 — CONTROL PID DIGITAL EN DOMINIO Z (con filtro derivativo)
% -------------------------------------------------------------------------
% Propósito  : Sintonizar un PIDF (PID con filtro derivativo) en dominio
%              z usando `pidtune`, partiendo de especificaciones (Mp, tp)
%              traducidas a (ωn, ζ, PM) para controlar la posición
%              angular del motor; validar márgenes y respuesta al escalón.
% Aplicación : Motor BDC (modelo 3x3, planta real), C = [0 0 1].
% Parámetros : Planta real (Ra=11, La=0.008, Kb=0.0014, Je=7.56e-4, Be=1e-5).
% Muestreo   : Ts = 1 ms (configurable en sección 4).
% Entradas   : Mp, tp.
% Salidas    : Parámetros C(z) (Kp, Ki, Kd, Tf), márgenes, respuesta al
%              escalón, bloques individuales para usar en pid_bdc_z_sim.slx.
% Doc        : docs/05_pid_digital.md
% -------------------------------------------------------------------------
% LEYENDA DE IMPLEMENTACIÓN
%   [PC]  : se ejecuta off-line en MATLAB (diseño/sintonía/validación).
%   [MCU] : pertenece al algoritmo que corre en el microcontrolador a
%           cada interrupción de muestreo (cada Ts segundos).
%
% En este script TODO es [PC]: el objetivo es OBTENER los coeficientes
% (Kp, Ki, Kd, Tf) que luego se programan en el microcontrolador. Lo único
% que se ejecuta en el [MCU] es la ECUACIÓN RECURSIVA del PIDF discreto:
%
%   [MCU]  e[k]  = r[k] - y[k];
%   [MCU]  P[k]  = Kp * e[k];
%   [MCU]  I[k]  = I[k-1] + Ki*Ts * e[k];                 % integral
%   [MCU]  D[k]  = (Tf/(Tf+Ts))*D[k-1] +
%                 (Kd/(Tf+Ts))*(e[k]-e[k-1]);            % derivativo filtrado
%   [MCU]  u[k]  = saturar(P[k] + I[k] + D[k], Umin, Umax);
%   [MCU]  aplicar_PWM(u[k]);
% =========================================================================

clear; clc; close all;

disp('=========================================================================');
disp('   Diseño Avanzado de Control Discreto PID - Posición de Motor DC        ');
disp('=========================================================================');

%% 1. ESPECIFICACIONES DE DISEÑO
disp('--- 1. Especificaciones de Diseño ---');
Mp = 0.40;  % Sobreimpulso máximo (40%)
tp = 10;   % Tiempo pico deseado (s)

% Cálculo de parámetros dominantes continuos a lazo cerrado
zeta = -log(Mp) / sqrt(pi^2 + (log(Mp))^2);
wn = pi / (tp * sqrt(1 - zeta^2));

sigma = zeta * wn;
wd = wn * sqrt(1 - zeta^2);

fprintf('Factor de amortiguamiento (zeta) : %.4f\n', zeta);
fprintf('Frecuencia natural (wn)          : %.4f rad/s\n', wn);

%% 2. EQUIVALENCIA FRECUENCIAL PARA DISEÑO PID
disp(' ');
disp('--- 2. Equivalencia Frecuencial para PID ---');
wc_deseada = wn; 
PM_rad = atan(2*zeta / sqrt(sqrt(1+4*zeta^4) - 2*zeta^2));
PM_deg = rad2deg(PM_rad);

fprintf('Frecuencia de Cruce objetivo (Wc): %.4f rad/s\n', wc_deseada);
fprintf('Margen de Fase objetivo (PM)   : %.2f grados\n', PM_deg);

%% 3. ELECCIÓN DEL TIEMPO DE MUESTREO (Ts) Y DISCRETIZACIÓN
disp(' ');
disp('--- 3. Justificación del Tiempo de Muestreo (Ts) ---');
% TEOREMA DE SHANNON Y REGLAS DE INGENIERÍA:
% Para control digital, la frecuencia de muestreo (fs) debe ser
% entre 10 y 30 veces la frecuencia natural del lazo cerrado (fn).
fn_hz = wn / (2*pi); % Frecuencia natural en Hertz
fprintf('Frecuencia natural del sistema (fn): %.4f Hz\n', fn_hz);

% Elegimos Ts = 1 ms -> fs = 1 kHz, muy por encima de la regla práctica.
Ts = 0.001; 
fs = 1/Ts;
fprintf('Tiempo de muestreo configurado (Ts) : %.3f s\n', Ts);
fprintf('Frecuencia de muestreo lograda (fs) : %.2f Hz (%.1fx fn)\n', fs, fs/fn_hz);

% Planta y discretización
Ra = 11; La = 0.008; Kb = 0.0014; Je = 0.000756; Be = 0.00001;
A = [-Ra/La, -Kb/La, 0 ; Kb/Je, -Be/Je, 0 ; 0, 1, 0]; 
B = [1/La ; 0 ; 0]; C = [0, 0, 1]; D = 0;

sys_planta_s = ss(A, B, C, D);
sys_planta_z = c2d(sys_planta_s, Ts, 'zoh');

%% 4. SÍNTESIS DEL CONTROLADOR PID DISCRETO (CON FILTRO CAUSAL)
% [PC] Esta sintonía corre UNA SOLA VEZ en MATLAB. Sus salidas
%      (Kp, Ki, Kd, Tf) son las constantes que se compilan dentro del [MCU].
disp(' ');
disp('--- 4. Controlador PID Discreto (Causal) ---');
opts = pidtuneOptions('PhaseMargin', PM_deg, 'DesignFocus', 'reference-tracking');
C_z = pidtune(sys_planta_z, 'PIDF', wc_deseada, opts);
C_z 

%% 5. ANÁLISIS TEÓRICO 1: LUGAR DE LAS RAÍCES EN PLANO Z (SEPARADO)
L_z = C_z * sys_planta_z;
sys_cl_z = feedback(L_z, 1);
p_cl = pole(sys_cl_z);

% Mapeo del polo S ideal al plano Z: z = e^{s*Ts}
s_ideal = -sigma + 1i*wd;
z_ideal = exp(s_ideal * Ts);

figure('Name', 'Lugar de las Raíces Discreto', 'Position', [50, 100, 1000, 500]);

% Gráfico Izquierdo: Vista Global
subplot(1,2,1);
rlocus(L_z); hold on;
zgrid; % Agrega la cuadrícula de amortiguamiento y frecuencia en Z
plot(real(p_cl), imag(p_cl), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', 'm');
plot(real(z_ideal), imag(z_ideal), 'o', 'MarkerSize', 8, 'LineWidth', 2, 'Color', 'k');
title('Lugar de las Raíces: Vista Global');
legend('RLocus', 'Polos CL PID', 'Polo Ideal', 'Location', 'best');

% Gráfico Derecho: Vista con Zoom a la dinámica dominante (cerca de z=1)
subplot(1,2,2);
rlocus(L_z); hold on;
zgrid;
plot(real(p_cl), imag(p_cl), 'x', 'MarkerSize', 12, 'LineWidth', 2.5, 'Color', 'm');
plot(real(z_ideal), imag(z_ideal), 'o', 'MarkerSize', 10, 'LineWidth', 2, 'Color', 'k');
% Aplicamos Zoom calculando los límites dinámicamente
axis([min(real(p_cl))-0.1, 1.1, -abs(imag(p_cl(1)))-0.2, abs(imag(p_cl(1)))+0.2]);
title('Lugar de las Raíces: Zoom Polos Dominantes');
xlabel('Eje Real'); ylab_str = ylabel('Eje Imaginario');

%% 6. ANÁLISIS TEÓRICO 2: RESPUESTA EN FRECUENCIA (BODE)
% Demostramos visualmente que logramos la Frecuencia de Cruce y el Margen de Fase
figure('Name', 'Análisis de Lazo Abierto (Bode)', 'Position', [100, 150, 600, 450]);
margin(L_z); grid on;
title(sprintf('Bode Lazo Abierto L(z) - Objetivo PM: %.1f\\circ a Wc: %.1f rad/s', PM_deg, wc_deseada));

%% 7. APLICACIÓN: RESPUESTA TEMPORAL COMPLETA
G_ideal_s = tf(wn^2, [1, 2*zeta*wn, wn^2]);
t_sim = 0:Ts:10*tp;
r_sim = ones(size(t_sim)); % Señal de referencia (Escalón)

figure('Name', 'Dinámica Temporal del Motor', 'Position', [150, 150, 1000, 600]);

% 7.1 Seguimiento de Posición
subplot(2,2,1);
[y_ideal, t_ideal] = step(G_ideal_s, t_sim);
plot(t_ideal, y_ideal, 'k--', 'LineWidth', 1.5); hold on;
[y_z, t_z] = step(sys_cl_z, t_sim);
plot(t_z, y_z, 'b-', 'LineWidth', 1.5);     
title('Seguimiento de Posición Angular (\theta)');
legend('2do Orden Continuo', 'Motor DC Control Digital', 'Location', 'SouthEast');
xlabel('Tiempo (s)'); ylabel('Posición (rad)');
grid on;

% 7.2 Esfuerzo de Control Discreto (Voltaje)
subplot(2,2,2);
sys_u_z = feedback(C_z, sys_planta_z); 
[u_val, t_val] = step(sys_u_z, t_sim);
stairs(t_val, u_val, 'r', 'LineWidth', 1.5);
title('Esfuerzo de Control Digital (Voltaje V_a)');
xlabel('Tiempo (s)'); ylabel('Voltaje retenido (V)');
grid on;

% 7.3 Dinámica del Error (Ref - Salida)
subplot(2,2,3);
error_z = 1 - y_z; % Error e(t) = r(t) - y(t)
plot(t_z, error_z, 'g-', 'LineWidth', 1.5); hold on;
plot(t_z, zeros(size(t_z)), 'k--');
title('Evolución de la Señal de Error e(t)');
xlabel('Tiempo (s)'); ylabel('Amplitud del Error');
grid on;

% 7.4 Mapa de Polos y Ceros en Lazo Cerrado
subplot(2,2,4);
pzmap(sys_cl_z); 
title('Mapa Polos/Ceros - Lazo Cerrado H(z)');
grid on;

%% 8. RESULTADOS NUMÉRICOS
info_real = stepinfo(sys_cl_z);
disp(' ');
disp('--- 5. Verificación de Resultados Numéricos ---');
fprintf('Sobreimpulso Esperado: %.2f %% | Obtenido: %.2f %%\n', Mp*100, info_real.Overshoot);
fprintf('Tiempo Pico Esperado : %.4f s  | Obtenido: %.4f s\n', tp, info_real.PeakTime);