% =========================================================================
% Cap. 06 — PID DIGITAL CON SATURACIÓN Y ANTI-WINDUP (back-calculation)
% -------------------------------------------------------------------------
% Propósito  : Comparar el mismo PID digital SIN y CON anti-windup ante
%              saturación del actuador (puente H ±24 V), usando back-
%              calculation. Script paramétrico: tiempo y gráficos se
%              auto-ajustan según `tp`.
% Aplicación : Motor BDC (modelo 3x3, planta real), C = [0 0 1].
% Parámetros : Planta real (Ra=11, La=0.008, Kb=0.0014, Je=7.56e-4, Be=1e-5);
%              saturación Vsat = ±24 V.
% Muestreo   : Ts = 1 ms.
% Entradas   : Mp, tp, Escalon_Ref (magnitud de referencia en rad).
% Salidas    : Figura con 4 subplots comparando respuesta sin/con AW,
%              esfuerzo de control, error e integrador.
% Doc        : docs/06_anti_windup.md
% -------------------------------------------------------------------------
% LEYENDA DE IMPLEMENTACIÓN
%   [PC]  : se ejecuta off-line en MATLAB (diseño/sintonía/validación).
%   [MCU] : pertenece al algoritmo que corre en el microcontrolador a
%           cada interrupción de muestreo (cada Ts segundos).
%
% En el bucle de simulación de la sección 4 se distinguen claramente las
% líneas marcadas como [MCU] (cómputo del PID + saturación + back-
% calculation) de las marcadas como [PC] (modelo del motor `Phi*x+Gamma*u`,
% que en la realidad NO se programa: es el motor físico el que evoluciona).
% =========================================================================

clear; clc; close all;

disp('=========================================================================');
disp('   Diseño PID Discreto: Efecto del Windup y Solución Anti-Windup         ');
disp('=========================================================================');

%% 1. PARÁMETROS CONFIGURABLES POR EL USUARIO
Mp = 0.6;          % Sobreimpulso deseado (Ej: 0.60 = 60%)
tp = 3;            % Tiempo pico deseado (s)
Escalon_Ref = 10;  % Magnitud de la referencia de posición (rad)

fprintf('>>> Configuración Actual: tp = %.2f s | Mp = %.0f%% <<<\n\n', tp, Mp*100);

%% 2. DISEÑO DEL PID IDEAL
% Conversión (Mp, tp) -> (zeta, wn) usando las fórmulas estándar del cap. 02
% y el margen de fase aproximado (sec 5.7 del .md).
zeta = -log(Mp) / sqrt(pi^2 + (log(Mp))^2);
wn = pi / (tp * sqrt(1 - zeta^2));
PM_deg = rad2deg(atan(2*zeta / sqrt(sqrt(1+4*zeta^4) - 2*zeta^2)));

% Planta Motor DC -- modelo 3x3 [ia, w, theta]^T  (sec 1.x del cap. 01)
Ra = 11; La = 0.008; Kb = 0.0014; Je = 0.000756; Be = 0.00001;
A = [-Ra/La, -Kb/La, 0 ; Kb/Je, -Be/Je, 0 ; 0, 1, 0]; 
B = [1/La ; 0 ; 0]; C = [0, 0, 1]; D = 0;
sys_planta_s = ss(A, B, C, D);

% Ts = 5 ms: sigue siendo >> tau_e (La/Ra ≈ 0.7 ms) y reduce 5× el costo
% del bucle escalar (N pasa de 240 k a 48 k para T_end = 80*tp).
Ts = 0.005;
sys_planta_z = c2d(sys_planta_s, Ts, 'zoh');

% Sintonización automática con pidtune fijando wn como ancho de banda objetivo
% y PM_deg como margen de fase requerido (ver doc cap. 05).
opts = pidtuneOptions('PhaseMargin', PM_deg, 'DesignFocus', 'reference-tracking');
ctrl = 'PIDF';
C_z = pidtune(sys_planta_z, ctrl, wn, opts);

% Extracción robusta: pidtune puede devolver objetos pid/pid2/pidstd con
% sólo el subconjunto de ganancias que aplica al tipo solicitado (p.ej. 'P'
% no tiene Ki/Kd/Tf, 'PI' no tiene Kd/Tf, 'PID' no tiene Tf, etc.). Las
% ganancias ausentes se ponen explícitamente en 0 para que el bucle PID
% (up + ui + ud con derivador filtrado) siga funcionando sin tocarse.
% Nota: `isprop` no funciona bien con objetos pid (sobrecarga de '()'),
% así que comparamos contra la lista devuelta por `properties`.
props = properties(C_z);
Kp = 0; Ki = 0; Kd = 0; Tf = 0;
if any(strcmp(props, 'Kp')), Kp = C_z.Kp; end
if any(strcmp(props, 'Ki')), Ki = C_z.Ki; end
if any(strcmp(props, 'Kd')), Kd = C_z.Kd; end
if any(strcmp(props, 'Tf')), Tf = C_z.Tf; end

% Tf = 0 haría que (Tf+Ts) = Ts (válido) pero a_d = 0: el derivador queda
% como diferencia simple Kd*(e[k]-e[k-1])/Ts. Si además Kd = 0, la rama
% derivativa queda inactiva. Sin riesgo de división por cero.

disp(['--- Parámetros del Controlador Sintonizado (' ctrl ') ---']);
fprintf('Kp: %.4f | Ki: %.4f | Kd: %.4f | Tf: %.5f\n', Kp, Ki, Kd, Tf);

%% 3. CONFIGURACIÓN DINÁMICA DE LA SIMULACIÓN
% Límite Físico del Motor
V_max = 24;  
V_min = -24;

% Ganancia Anti-Windup (Back-Calculation)
% Heurística simple: Kaw = Kp/10  (ver Eq. de descarga del integrador,
% sec 6.3 del .md). Constante de tiempo de descarga T_aw = 1/Kaw.
% Alternativa: Kaw = sqrt(Ki*Kd) (media geométrica de T_i, T_d).
if Ki == 0
    Kaw = 0;
else
    Kaw = Kp/10;
end 

% Horizonte de simulación automático
% Nota: usamos 80*tp (no 8*tp) porque la planta tiene constante mecánica
% Je/Be ≈ 75 s y la convergencia final del caso CON AW depende de la
% acción integral residual; con horizontes cortos parecería haber error
% estacionario cuando en realidad sólo es convergencia lenta.
T_end = 80 * tp; 
t_sim = 0:Ts:T_end; 
N = length(t_sim);

Referencia = Escalon_Ref * ones(1, N); 

Phi = sys_planta_z.A; Gamma = sys_planta_z.B; C_mat = sys_planta_z.C;

% Inicialización
x_no_aw = zeros(3, N); y_no_aw = zeros(1, N); u_no_aw = zeros(1, N); u_calc_no_aw = zeros(1,N);
ui_no_aw = 0; ud_no_aw = 0; e_prev_no_aw = 0; ui_hist_no_aw = zeros(1, N);

x_aw = zeros(3, N); y_aw = zeros(1, N); u_aw = zeros(1, N); u_calc_aw = zeros(1, N);  
ui_aw = 0; ud_aw = 0; e_prev_aw = 0; ui_hist_aw = zeros(1, N); 

% Coeficientes precalculados del derivador filtrado (constantes en el bucle)
a_d = Tf / (Tf + Ts);
b_d = Kd / (Tf + Ts);

%% 4. BUCLE DE SIMULACIÓN NO LINEAL
% Cada iteración del bucle equivale a UNA interrupción de muestreo del
% microcontrolador (período Ts). Las líneas [MCU] son las que se traducen
% a código C dentro de la ISR del timer; las líneas [PC] modelan al motor
% (en hardware real las ejecuta la planta física, no el firmware).
for k = 1:N-1
    
    % --- PID SIN ANTI-WINDUP ---
    % Eq. (5.x): u[k] = up[k] + ui[k] + ud[k]
    y_no_aw(k) = C_mat * x_no_aw(:, k);                                     % [MCU] leer encoder/ADC -> y[k]
    error_no_aw = Referencia(k) - y_no_aw(k);                               % [MCU] e[k] = r[k] - y[k]
    
    up_no_aw = Kp * error_no_aw;                                            % [MCU] rama proporcional
    ui_no_aw = ui_no_aw + Ki * Ts * error_no_aw;                            % [MCU] integración (acumulador)
    ud_no_aw = a_d * ud_no_aw + b_d * (error_no_aw - e_prev_no_aw);         % [MCU] derivador filtrado
    e_prev_no_aw = error_no_aw;                                             % [MCU] guardar e[k-1]
    
    v_calc_no = up_no_aw + ui_no_aw + ud_no_aw;                             % [MCU] señal pedida por el PID
    u_calc_no_aw(k) = v_calc_no;
    
    v_sat_no = max(min(v_calc_no, V_max), V_min);                           % [MCU] saturación por software (o la hace el driver)
    u_no_aw(k) = v_sat_no;                                                  % [MCU] escribir PWM
    
    ui_hist_no_aw(k) = ui_no_aw;                                            % [PC]  logging para diagnóstico (gráfico 3)
    x_no_aw(:, k+1) = Phi * x_no_aw(:, k) + Gamma * u_no_aw(k);             % [PC]  modelo del motor (la planta física en HW)
    
    % --- PID CON ANTI-WINDUP (back-calculation) ---
    % Eq. (6.x): ui[k+1] = ui[k] + Ki*Ts*e + Kaw*Ts*(u_sat - u_calc)
    y_aw(k) = C_mat * x_aw(:, k);                                           % [MCU] leer sensor
    error_aw = Referencia(k) - y_aw(k);                                     % [MCU]
    
    up_aw = Kp * error_aw;                                                  % [MCU]
    ud_aw = a_d * ud_aw + b_d * (error_aw - e_prev_aw);                     % [MCU]
    ui_aw = ui_aw + Ki * Ts * error_aw;                                     % [MCU]
    e_prev_aw = error_aw;                                                   % [MCU]
  
    v_calc = up_aw + ui_aw + ud_aw;                                         % [MCU] señal pedida
    u_calc_aw(k) = v_calc;
    
    v_sat = max(min(v_calc, V_max), V_min);                                 % [MCU] saturación
    u_aw(k) = v_sat;                                                        % [MCU] escribir PWM
    
    % Diferencia entre lo entregado y lo pedido: NEGATIVA cuando satura por arriba
    % -> resta del integrador, descargándolo (anti-windup)
    error_aw_term = v_sat - v_calc;                                         % [MCU] término de back-calculation
    ui_aw = ui_aw + Kaw * Ts * error_aw_term;                               % [MCU] integrador con anti-windup
    
    ui_hist_aw(k) = ui_aw;                                                  % [PC]  logging
    x_aw(:, k+1) = Phi * x_aw(:, k) + Gamma * u_aw(k);                      % [PC]  modelo del motor
end

% Acomodo del último índice
y_no_aw(N) = C_mat * x_no_aw(:, N); y_aw(N) = C_mat * x_aw(:, N);
ui_hist_no_aw(N) = ui_hist_no_aw(N-1); ui_hist_aw(N) = ui_hist_aw(N-1);
u_calc_no_aw(N) = u_calc_no_aw(N-1); 

%% 5. ANÁLISIS GRÁFICO
% Decimación para graficar: ~5000 puntos por traza es más que suficiente
% visualmente y evita que MATLAB renderice cientos de miles de vértices.
MAX_PTS = 5000;
dec = max(1, floor(N / MAX_PTS));
idx = 1:dec:N;
tp_plot       = t_sim(idx);
Ref_p         = Referencia(idx);
y_no_aw_p     = y_no_aw(idx);
y_aw_p        = y_aw(idx);
u_no_aw_p     = u_no_aw(idx);
u_aw_p        = u_aw(idx);
u_calc_no_p   = u_calc_no_aw(idx);
ui_hist_no_p  = ui_hist_no_aw(idx);
ui_hist_aw_p  = ui_hist_aw(idx);

fig = figure('Name', sprintf('Saturación y AW (tp = %.2fs, Mp = %.0f%%)', tp, Mp*100), 'Position', [50, 50, 1000, 850], 'Color', 'w');

% --- VENTANA DE VISUALIZACIÓN ADAPTATIVA -------------------------------
% El bucle simula hasta T_end = 80*tp para que el caso CON AW cierre el
% último tramo vía integral lenta, pero el transitorio de interés
% (saturación y windup) ocurre en los primeros ~10*tp. Hacemos zoom
% automático a esa ventana.
x_zoom = min(T_end, max(10*tp, 4));     % al menos 4 s o 10*tp
x_zoom = min(T_end, max(10*tp, 4));     % al menos 4 s o 10*tp

% Límites Y de posición: usan AMBAS trayectorias y son simétricos al signo
% de la referencia para soportar Escalon_Ref negativo. Margen proporcional
% al sobreimpulso teórico Mp para que un Mp pequeño no aplaste el gráfico
% y un Mp grande no recorte el pico.
y_min_data = min([0, Escalon_Ref, min(y_no_aw), min(y_aw)]);
y_max_data = max([0, Escalon_Ref, max(y_no_aw), max(y_aw)]);
y_margin   = max(0.05*abs(Escalon_Ref), 0.5*Mp*abs(Escalon_Ref));
y_lim_pos  = [y_min_data - y_margin, y_max_data + y_margin];

% --- GRÁFICO 1: POSICIÓN ---
ax1 = subplot(3,1,1);
plot(tp_plot, Ref_p, 'k--', 'LineWidth', 2); hold on;
% Banda de tolerancia ±5 % alrededor de la referencia (criterio de asentamiento)
band = 0.05 * abs(Escalon_Ref);
patch([tp_plot fliplr(tp_plot)], ...
      [Ref_p+band fliplr(Ref_p-band)], ...
      [0.6 0.8 0.6], 'EdgeColor','none', 'FaceAlpha',0.15, 'HandleVisibility','off');
plot(tp_plot, y_no_aw_p, 'Color', [0.85 0.33 0.10], 'LineWidth', 2.5); 
plot(tp_plot, y_aw_p, 'Color', [0 0.45 0.74], 'LineWidth', 2.5);       
title(sprintf('1. Salida: Posición Angular (tp = %.2f s, Mp = %.0f%%, ref = %g rad)', tp, Mp*100, Escalon_Ref), 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Posición (rad)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Referencia', 'PID Clásico (Con Windup)', 'PID con Anti-Windup', 'Location', 'SouthEast', 'FontSize', 11);
grid on; ax1.GridAlpha = 0.3;
xline(tp, 'm:', 'tp Teórico', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5, 'HandleVisibility','off');
ylim(y_lim_pos);

% --- GRÁFICO 2: ESFUERZO DE CONTROL (VOLTAJE) ---
ax2 = subplot(3,1,2);

% Límites del eje Y robustos al derivative kick (que dispara u_calc lejos
% del rango de saturación). Se acotan a 4*V_sat para evitar que un pico
% numérico aplaste visualmente el resto de la traza.
max_v_plot = max([V_max + 5, min(max(u_calc_no_aw) + 5, V_max * 4)]);
min_v_plot = min([V_min - 5, max(min(u_calc_no_aw) - 5, V_min * 4)]);

patch([0 T_end T_end 0], [V_max V_max max_v_plot max_v_plot], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off'); hold on;
patch([0 T_end T_end 0], [V_min V_min min_v_plot min_v_plot], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off'); hold on;

plot(tp_plot, u_calc_no_p, ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); 
plot(tp_plot, u_no_aw_p, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 3);        
plot(tp_plot, u_aw_p, '-', 'Color', [0 0.45 0.74], 'LineWidth', 2);              

yline(V_max, 'k-', 'Límite (+24V)', 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(V_min, 'k-', 'Límite (-24V)', 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
ylim([min_v_plot, max_v_plot]); 
title('2. Esfuerzo de Control: Voltaje Aplicado al Motor', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Voltaje (V)', 'FontSize', 11, 'FontWeight', 'bold');
legend('PID pide', 'Real Saturado', 'Real PID + AW', 'Location', 'NorthEast', 'FontSize', 11);
grid on; ax2.GridAlpha = 0.3;

% --- GRÁFICO 3: MEMORIA DEL ACUMULADOR INTEGRAL ---
ax3 = subplot(3,1,3);
plot(tp_plot, ui_hist_no_p, 'Color', [0.85 0.33 0.10], 'LineWidth', 2.5); hold on;
plot(tp_plot, ui_hist_aw_p, 'Color', [0 0.45 0.74], 'LineWidth', 2.5);
yline(0, 'k-', 'HandleVisibility', 'off');

fill([tp_plot fliplr(tp_plot)], [ui_hist_no_p fliplr(ui_hist_aw_p)], [0.85 0.33 0.10], 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

title('3. Diagnóstico: Memoria del Acumulador Integral (u_i)', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('Tiempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Esfuerzo Integral', 'FontSize', 11, 'FontWeight', 'bold');
legend('Windup', 'Anti-Windup', 'Location', 'NorthEast', 'FontSize', 11);
grid on; ax3.GridAlpha = 0.3;

% Eje X enlazado entre los tres subplots
linkaxes([ax1, ax2, ax3], 'x');
xlim([0, x_zoom]);