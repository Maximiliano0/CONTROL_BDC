% =========================================================================
% Control Digital PID (Dominio Z) con SATURACIÓN y ANTI-WINDUP
% Aplicación: Control de POSICIÓN angular de Motor BDC (modelo 3x3)
% Driver: puente H con saturación a +/- 24 V
% Parámetros: planta real (Ra=11, La=0.008, Kb=0.0014, Je=7.56e-4, Be=1e-5)
% Script paramétrico: auto-ajusta tiempo y gráficos según 'tp'
% =========================================================================

clear; clc; close all;

disp('=========================================================================');
disp('   Diseño PID Discreto: Efecto del Windup y Solución Anti-Windup         ');
disp('=========================================================================');

%% 1. PARÁMETROS CONFIGURABLES POR EL USUARIO
% ¡Modifica estos valores para ver cómo se ajusta todo automáticamente!
Mp = 0.3;          % Sobreimpulso deseado (Ej: 0.30 = 30%)
tp = 5;            % Tiempo pico deseado (s)
Escalon_Ref = 10;  % Magnitud de la referencia de posición (rad)

fprintf('>>> Configuración Actual: tp = %.2f s | Mp = %.0f%% <<<\n\n', tp, Mp*100);

%% 2. DISEÑO DEL PID IDEAL
zeta = -log(Mp) / sqrt(pi^2 + (log(Mp))^2);
wn = pi / (tp * sqrt(1 - zeta^2));
PM_deg = rad2deg(atan(2*zeta / sqrt(sqrt(1+4*zeta^4) - 2*zeta^2)));

% Planta Motor DC
Ra = 11; La = 0.008; Kb = 0.0014; Je = 0.000756; Be = 0.00001;
A = [-Ra/La, -Kb/La, 0 ; Kb/Je, -Be/Je, 0 ; 0, 1, 0]; 
B = [1/La ; 0 ; 0]; C = [0, 0, 1]; D = 0;
sys_planta_s = ss(A, B, C, D);

Ts = 0.001; % Muestreo a 1ms
sys_planta_z = c2d(sys_planta_s, Ts, 'zoh');

% Sintonización
opts = pidtuneOptions('PhaseMargin', PM_deg, 'DesignFocus', 'reference-tracking');
C_z = pidtune(sys_planta_z, 'PIDF', wn, opts);

Kp = C_z.Kp; Ki = C_z.Ki; Kd = C_z.Kd; Tf = C_z.Tf; 
disp('--- Parámetros del PID Sintonizado ---');
fprintf('Kp: %.4f | Ki: %.4f | Kd: %.4f | Tf: %.5f\n', Kp, Ki, Kd, Tf);

%% 3. CONFIGURACIÓN DINÁMICA DE LA SIMULACIÓN
% Límite Físico del Motor
V_max = 24;  
V_min = -24;

% Ganancia Anti-Windup (Back-Calculation)
Kaw = Kp/10; 

% Horizonte de simulación automático
T_end = 8 * tp; 
t_sim = 0:Ts:T_end; 
N = length(t_sim);

Referencia = Escalon_Ref * ones(1, N); 

Phi = sys_planta_z.A; Gamma = sys_planta_z.B; C_mat = sys_planta_z.C;

% Inicialización
x_no_aw = zeros(3, N); y_no_aw = zeros(1, N); u_no_aw = zeros(1, N); u_calc_no_aw = zeros(1,N);
ui_no_aw = 0; ud_no_aw = 0; e_prev_no_aw = 0; ui_hist_no_aw = zeros(1, N);

x_aw = zeros(3, N); y_aw = zeros(1, N); u_aw = zeros(1, N); u_calc_aw = zeros(1, N);  
ui_aw = 0; ud_aw = 0; e_prev_aw = 0; ui_hist_aw = zeros(1, N); 

%% 4. BUCLE DE SIMULACIÓN NO LINEAL
for k = 1:N-1
    
    % --- PID SIN ANTI-WINDUP ---
    y_no_aw(k) = C_mat * x_no_aw(:, k);
    error_no_aw = Referencia(k) - y_no_aw(k);
    
    up_no_aw = Kp * error_no_aw;
    ui_no_aw = ui_no_aw + Ki * Ts * error_no_aw;
    ud_no_aw = (Tf / (Tf + Ts)) * ud_no_aw + (Kd / (Tf + Ts)) * (error_no_aw - e_prev_no_aw);
    e_prev_no_aw = error_no_aw;
    
    v_calc_no = up_no_aw + ui_no_aw + ud_no_aw;
    u_calc_no_aw(k) = v_calc_no;
    
    v_sat_no = max(min(v_calc_no, V_max), V_min);
    u_no_aw(k) = v_sat_no;
    
    ui_hist_no_aw(k) = ui_no_aw; 
    x_no_aw(:, k+1) = Phi * x_no_aw(:, k) + Gamma * u_no_aw(k);
    
    % --- PID CON ANTI-WINDUP ---
    y_aw(k) = C_mat * x_aw(:, k);
    error_aw = Referencia(k) - y_aw(k);
    
    up_aw = Kp * error_aw;
    ud_aw = (Tf / (Tf + Ts)) * ud_aw + (Kd / (Tf + Ts)) * (error_aw - e_prev_aw);
    e_prev_aw = error_aw;
    
    v_calc = up_aw + ui_aw + ud_aw;
    u_calc_aw(k) = v_calc;
    
    v_sat = max(min(v_calc, V_max), V_min);
    u_aw(k) = v_sat;
    
    error_aw_term = v_sat - v_calc;
    ui_aw = ui_aw + Ki * Ts * error_aw + Kaw * Ts * error_aw_term;
    
    ui_hist_aw(k) = ui_aw; 
    x_aw(:, k+1) = Phi * x_aw(:, k) + Gamma * u_aw(k);
end

% Acomodo del último índice
y_no_aw(N) = C_mat * x_no_aw(:, N); y_aw(N) = C_mat * x_aw(:, N);
ui_hist_no_aw(N) = ui_hist_no_aw(N-1); ui_hist_aw(N) = ui_hist_aw(N-1);
u_calc_no_aw(N) = u_calc_no_aw(N-1); 

%% 5. ANÁLISIS GRÁFICO AUTO-AJUSTABLE
fig = figure('Name', sprintf('Saturación y AW (tp = %.2fs)', tp), 'Position', [50, 50, 1000, 850], 'Color', 'w');

% --- GRÁFICO 1: POSICIÓN ---
ax1 = subplot(3,1,1);
plot(t_sim, Referencia, 'k--', 'LineWidth', 2); hold on;
plot(t_sim, y_no_aw, 'Color', [0.85 0.33 0.10], 'LineWidth', 2.5); 
plot(t_sim, y_aw, 'Color', [0 0.45 0.74], 'LineWidth', 2.5);       
title(sprintf('1. Salida: Posición Angular (Objetivo tp = %.2f s)', tp), 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Posición (rad)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Referencia', 'PID Clásico (Con Windup)', 'PID con Anti-Windup', 'Location', 'SouthEast', 'FontSize', 11);
grid on; ax1.GridAlpha = 0.3;
xline(tp, 'm:', 'tp Teórico', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5, 'HandleVisibility','off');
ylim([0, Escalon_Ref + max(abs(y_no_aw - Escalon_Ref))*1.1]);

% --- GRÁFICO 2: ESFUERZO DE CONTROL (VOLTAJE) ---
ax2 = subplot(3,1,2);

% LÓGICA DE LÍMITES SÚPER ROBUSTA (A prueba de Derivative Kick)
% Garantizamos que min_v_plot SIEMPRE sea menor estricto que max_v_plot
max_v_plot = max([V_max + 5, min(max(u_calc_no_aw) + 5, V_max * 4)]);
min_v_plot = min([V_min - 5, max(min(u_calc_no_aw) - 5, V_min * 4)]);

patch([0 T_end T_end 0], [V_max V_max max_v_plot max_v_plot], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off'); hold on;
patch([0 T_end T_end 0], [V_min V_min min_v_plot min_v_plot], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off'); hold on;

plot(t_sim, u_calc_no_aw, ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); 
plot(t_sim, u_no_aw, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 3);        
plot(t_sim, u_aw, '-', 'Color', [0 0.45 0.74], 'LineWidth', 2);              

yline(V_max, 'k-', 'Límite (+24V)', 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(V_min, 'k-', 'Límite (-24V)', 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'HandleVisibility', 'off');
yline(0, 'k-', 'HandleVisibility', 'off');
ylim([min_v_plot, max_v_plot]); 
title('2. Esfuerzo de Control: Voltaje Aplicado al Motor', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Voltaje (V)', 'FontSize', 11, 'FontWeight', 'bold');
legend('PID pide', 'Real Saturado', 'Real PID + AW', 'Location', 'NorthEast', 'FontSize', 11);
grid on; ax2.GridAlpha = 0.3;

% --- GRÁFICO 3: EL "CULPABLE" (MEMORIA INTEGRAL) ---
ax3 = subplot(3,1,3);
plot(t_sim, ui_hist_no_aw, 'Color', [0.85 0.33 0.10], 'LineWidth', 2.5); hold on;
plot(t_sim, ui_hist_aw, 'Color', [0 0.45 0.74], 'LineWidth', 2.5);
yline(0, 'k-', 'HandleVisibility', 'off');

fill([t_sim fliplr(t_sim)], [ui_hist_no_aw fliplr(ui_hist_aw)], [0.85 0.33 0.10], 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');

title('3. Diagnóstico: Memoria del Acumulador Integral (u_i)', 'FontSize', 13, 'FontWeight', 'bold');
xlabel('Tiempo (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Esfuerzo Integral', 'FontSize', 11, 'FontWeight', 'bold');
legend('Windup', 'Anti-Windup', 'Location', 'NorthEast', 'FontSize', 11);
grid on; ax3.GridAlpha = 0.3;

% ENLACE DE EJES X (Zoom dinámico para la clase)
linkaxes([ax1, ax2, ax3], 'x');
xlim([0, T_end]);