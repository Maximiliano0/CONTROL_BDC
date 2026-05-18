% =========================================================================
% Cap. 09 — CONTROL LQR DISCRETO (CUADRÁTICO ÓPTIMO, 3x3, planta real)
% -------------------------------------------------------------------------
% Propósito  : Diseñar control LQR discreto con `dlqr` usando la regla de
%              Bryson para fijar Q y R; comparar tres configuraciones
%              (baseline, "rápido", "suave") y contrastarlas con el pole-
%              placement del cap. 07. Minimiza
%                  J = Σ_k ( x[k]ᵀ Q x[k] + u[k]ᵀ R u[k] ).
% Aplicación : Motor BDC (modelo 3x3, planta real), posición angular θ.
% Parámetros : Planta real (Ra=11, La=0.008, Kb=0.0014, Je=7.56e-4, Be=1e-5);
%              saturación V_max = ±24 V; rangos Bryson:
%              ia_max=5 A, we_max=200 rad/s, theta_max=20°.
% Muestreo   : Ts = 10 ms.
% Entradas   : Ts, Ref_grados, V_max, ia_max, we_max, theta_max, Mp_pp, tp_pp.
% Salidas    : K_A/K_B/K_C (LQR) y K_pp (pole-placement), métricas (Mp,
%              ts, J) en consola y figura comparativa de las 4 estrategias.
% Doc        : docs/09_control_lqr.md
% -------------------------------------------------------------------------
% Comparamos:
%   - LQR (A) baseline       : regla de Bryson sobre rangos máximos.
%   - LQR (B) "barato en u"  : R bajo  → respuesta rápida, mucho voltaje.
%   - LQR (C) "caro en u"    : R alto  → respuesta suave, poco voltaje.
%   - Pole-placement cap. 07 : referencia visual.
% -------------------------------------------------------------------------
% LEYENDA DE IMPLEMENTACIÓN
%   [PC]  : se ejecuta off-line en MATLAB (diseño/sintonía/validación).
%   [MCU] : pertenece al algoritmo que corre en el microcontrolador a
%           cada interrupción de muestreo (cada Ts segundos).
%
% Todo el cálculo de Q, R, dlqr/DARE y de las ganancias K_A, K_B, K_C, Kdc
% es [PC]: SOLO se hace una vez. Una vez elegida la sintonía (por ejemplo
% K_A y Kdc_A), el firmware del [MCU] ejecuta en cada ISR exactamente lo
% mismo que el pole-placement del cap. 07:
%   [MCU]  x[k] = leer_estados();                 % (ia, w, theta)
%   [MCU]  u[k] = -K*x[k] + Kdc*r[k];             % ley LQR
%   [MCU]  u[k] = saturar(u[k], Umin, Umax);
%   [MCU]  aplicar_PWM(u[k]);
% (La función local `sim_loop` modela esto: las líneas marcadas como
%  [MCU] dentro de ella son las que se traducen a código C.)
% =========================================================================

clear; clc; close all;

disp('=========================================================================');
disp('   Control Óptimo Cuadrático (LQR) Discreto - Posición del Motor BDC     ');
disp('=========================================================================');

%% 1. PARÁMETROS CONFIGURABLES POR EL USUARIO
Ts          = 0.01;     % Tiempo de muestreo (s)
Ref_grados  = 10;       % Referencia de posición (Grados)
V_max       =  24;      % Saturación del puente H (V)
V_min       = -24;

% Rangos máximos "tolerables" para la regla de Bryson
ia_max      = 5;                       % A   (corriente)
we_max      = 200;                     % rad/s
theta_max   = deg2rad(20);             % rad (~ 20°)
u_max       = V_max;                   % V

% Especificaciones SOLO para el pole-placement de referencia
Mp_pp = 0.10;  tp_pp = 1;

Ref_rad = deg2rad(Ref_grados);
fprintf('--- 1. Configuración ---\n');
fprintf('Ts = %.3f s   Ref = %.1f° (%.4f rad)   Saturación = ±%.0f V\n\n', ...
        Ts, Ref_grados, Ref_rad, V_max);

%% 2. PLANTA (CONTINUO -> DISCRETO ZOH)
Ra = 11; La = 0.008; Kb = 0.0014; Je = 0.000756; Be = 0.00001;
A  = [-Ra/La, -Kb/La, 0 ;  Kb/Je, -Be/Je, 0 ;  0, 1, 0];
B  = [1/La ; 0 ; 0];
C  = [0, 0, 1];
D  = 0;

sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts, 'zoh');
Phi   = sys_d.A;  Gamma = sys_d.B;  Cd = sys_d.C;
n     = size(Phi,1);

%% 3. REGLA DE BRYSON: Q y R BASE
% Q_ii = 1 / x_i_max^2     R = 1 / u_max^2
% Esto normaliza para que cada término de J esté en [0, 1] cuando el
% estado/control alcanzan su valor máximo aceptable.
Q_bryson = diag([1/ia_max^2, 1/we_max^2, 1/theta_max^2]);
R_bryson = 1/u_max^2;

fprintf('--- 2. Pesos por regla de Bryson ---\n');
disp('Q ='); disp(Q_bryson);
fprintf('R = %.4g\n\n', R_bryson);

%% 4. TRES SINTONÍAS LQR + UN POLE-PLACEMENT DE REFERENCIA
% Cada llamada a dlqr resuelve la DARE (Eq. 9.3 del .md):
%   P = Phi'*P*Phi - Phi'*P*Gamma*(R+Gamma'*P*Gamma)^-1*Gamma'*P*Phi + Q
% y devuelve K = (R+Gamma'*P*Gamma)^-1 * Gamma'*P*Phi.

% (A) LQR baseline (Bryson directo)
[Klqr_A, ~, P_A] = dlqr(Phi, Gamma, Q_bryson, R_bryson);

% (B) LQR "rápido": penalizamos 100x más el error de theta -> Q33 grande
%     -> K crece, polos del lazo cerrado más rápidos, más voltaje pico
Q_B = Q_bryson;  Q_B(3,3) = Q_B(3,3) * 100;
[Klqr_B, ~, P_B] = dlqr(Phi, Gamma, Q_B, R_bryson);

% (C) LQR "suave": penalizamos 100x más el esfuerzo -> R grande
%     -> K disminuye, polos más lentos, voltaje pequeño (no satura)
R_C = R_bryson * 100;
[Klqr_C, ~, P_C] = dlqr(Phi, Gamma, Q_bryson, R_C);

% (D) Pole-placement de referencia (idéntico al cap. 07) para comparar
zeta_pp = -log(Mp_pp) / sqrt(pi^2 + log(Mp_pp)^2);
wn_pp   = pi / (tp_pp * sqrt(1 - zeta_pp^2));
sigma_pp = zeta_pp * wn_pp;  wd_pp = wn_pp * sqrt(1 - zeta_pp^2);
P_pp    = exp([-sigma_pp + 1i*wd_pp, -sigma_pp - 1i*wd_pp, -10*sigma_pp] * Ts);
Kpp     = place(Phi, Gamma, P_pp);

% Pre-compensadores Kdc para que cada estrategia siga la referencia
Kdc = @(K) 1 / (Cd * ((eye(n) - (Phi - Gamma*K)) \ Gamma));
Kdc_A = Kdc(Klqr_A);  Kdc_B = Kdc(Klqr_B);
Kdc_C = Kdc(Klqr_C);  Kdc_pp = Kdc(Kpp);

fprintf('--- 3. Ganancias resultantes ---\n');
fprintf('LQR (A) baseline : K = [%.4f, %.4f, %.4f]   Kdc = %.4f\n', Klqr_A, Kdc_A);
fprintf('LQR (B) rápido   : K = [%.4f, %.4f, %.4f]   Kdc = %.4f\n', Klqr_B, Kdc_B);
fprintf('LQR (C) suave    : K = [%.4f, %.4f, %.4f]   Kdc = %.4f\n', Klqr_C, Kdc_C);
fprintf('Pole-placement   : K = [%.4f, %.4f, %.4f]   Kdc = %.4f\n\n', Kpp, Kdc_pp);

%% 5. SIMULACIÓN COMPARADA (con saturación de actuador)
t_end = 4 * tp_pp;
t_sim = 0:Ts:t_end;
N     = length(t_sim);
Ref   = Ref_rad * ones(1, N);

simulate = @(K, Kdc_val) sim_loop(Phi, Gamma, Cd, K, Kdc_val, Ref, V_min, V_max);
[y_A, u_A, J_A] = simulate(Klqr_A, Kdc_A);
[y_B, u_B, J_B] = simulate(Klqr_B, Kdc_B);
[y_C, u_C, J_C] = simulate(Klqr_C, Kdc_C);
[y_pp, u_pp, ~] = simulate(Kpp,    Kdc_pp);

%% 6. GRÁFICOS COMPARATIVOS
col_A  = [0    0.45 0.74];
col_B  = [0.85 0.33 0.10];
col_C  = [0.47 0.67 0.19];
col_pp = [0.49 0.18 0.56];

figure('Name', sprintf('LQR Comparado (Ref %.1f°)', Ref_grados), ...
       'Position', [60, 60, 1100, 750], 'Color', 'w');

% --- 6.1 Posición ---
ax1 = subplot(2,2,1);
plot(t_sim, rad2deg(Ref), 'k--', 'LineWidth', 2); hold on;
plot(t_sim, rad2deg(y_A),  'Color', col_A,  'LineWidth', 2);
plot(t_sim, rad2deg(y_B),  'Color', col_B,  'LineWidth', 2);
plot(t_sim, rad2deg(y_C),  'Color', col_C,  'LineWidth', 2);
plot(t_sim, rad2deg(y_pp), 'Color', col_pp, 'LineWidth', 1.5, 'LineStyle','--');
title('1. Posición \theta');
xlabel('Tiempo (s)'); ylabel('\theta (°)');
legend('Ref','LQR (A) baseline','LQR (B) rápido','LQR (C) suave','Pole-placement', ...
       'Location','SouthEast');
grid on; ax1.GridAlpha = 0.3;

% --- 6.2 Esfuerzo de control ---
ax2 = subplot(2,2,2);
stairs(t_sim, u_A,  'Color', col_A,  'LineWidth', 1.5); hold on;
stairs(t_sim, u_B,  'Color', col_B,  'LineWidth', 1.5);
stairs(t_sim, u_C,  'Color', col_C,  'LineWidth', 1.5);
stairs(t_sim, u_pp, 'Color', col_pp, 'LineWidth', 1.2, 'LineStyle','--');
yline( V_max, 'k-', '+24 V', 'HandleVisibility','off');
yline( V_min, 'k-', '-24 V', 'HandleVisibility','off');
yline(0, 'k:', 'HandleVisibility','off');
title('2. Esfuerzo de control (V_a)');
xlabel('Tiempo (s)'); ylabel('V');
legend('A','B','C','PP','Location','NorthEast');
grid on; ax2.GridAlpha = 0.3;

% --- 6.3 Costo acumulado J(t) ---
ax3 = subplot(2,2,3);
plot(t_sim, J_A,  'Color', col_A,  'LineWidth', 2); hold on;
plot(t_sim, J_B,  'Color', col_B,  'LineWidth', 2);
plot(t_sim, J_C,  'Color', col_C,  'LineWidth', 2);
title('3. Costo acumulado  J(t) = \Sigma (x^TQx + u^TRu)   con Q_A, R_A');
xlabel('Tiempo (s)'); ylabel('J');
legend('A baseline','B rápido','C suave','Location','SouthEast');
grid on; ax3.GridAlpha = 0.3;

% --- 6.4 Polos en plano Z ---
ax4 = subplot(2,2,4);
theta = linspace(0,2*pi,200);
plot(cos(theta), sin(theta), 'k-', 'LineWidth', 1); hold on; axis equal;
plot(real(P_A),  imag(P_A),  'x', 'Color', col_A,  'MarkerSize', 12, 'LineWidth', 2);
plot(real(P_B),  imag(P_B),  'x', 'Color', col_B,  'MarkerSize', 12, 'LineWidth', 2);
plot(real(P_C),  imag(P_C),  'x', 'Color', col_C,  'MarkerSize', 12, 'LineWidth', 2);
plot(real(P_pp), imag(P_pp), 's', 'Color', col_pp, 'MarkerSize', 10, 'LineWidth', 1.5);
xline(0,'k:','HandleVisibility','off'); yline(0,'k:','HandleVisibility','off');
title('4. Polos de lazo cerrado (\Phi - \Gamma K)');
xlabel('Re'); ylabel('Im');
legend('|z| = 1','A','B','C','PP','Location','SouthWest');
grid on; ax4.GridAlpha = 0.3;
xlim([-1.2 1.2]); ylim([-1.2 1.2]);

linkaxes([ax1, ax2, ax3], 'x'); xlim(ax1, [0, t_end]);

%% 7. REPORTE NUMÉRICO
fprintf('--- 4. Métricas (con saturación a ±%.0f V) ---\n', V_max);
report('LQR (A) baseline', y_A,  u_A,  J_A,  t_sim, Ref_rad);
report('LQR (B) rápido',   y_B,  u_B,  J_B,  t_sim, Ref_rad);
report('LQR (C) suave',    y_C,  u_C,  J_C,  t_sim, Ref_rad);
report('Pole-placement',   y_pp, u_pp, [],   t_sim, Ref_rad);

%% =====================================================================
%  FUNCIONES LOCALES
%  =====================================================================
function [y, u, Jacum] = sim_loop(Phi, Gamma, Cd, K, Kdc, Ref, umin, umax)
    n = size(Phi,1);  N = length(Ref);
    x = zeros(n, N);  y = zeros(1, N);  u = zeros(1, N);
    Q_ref = diag([1/5^2, 1/200^2, 1/deg2rad(20)^2]);   % [PC] pesos para reportar J
    R_ref = 1/24^2;
    Jacum = zeros(1, N);
    for k = 1:N-1
        y(k)     = Cd * x(:,k);                                  % [MCU] leer sensores
        u_calc   = -K * x(:,k) + Kdc * Ref(k);                   % [MCU] ley LQR u = -K*x + Kdc*r
        u(k)     = max(min(u_calc, umax), umin);                 % [MCU] saturación por software
        x(:,k+1) = Phi * x(:,k) + Gamma * u(k);                  % [PC]  modelo del motor (la planta real en HW)
        Jacum(k+1) = Jacum(k) + x(:,k)'*Q_ref*x(:,k) + u(k)*R_ref*u(k); % [PC] métrica de coste off-line
    end
    y(N) = Cd * x(:,N);   u(N) = u(N-1);
end

function report(name, y, u, J, t, Ref_rad)
    info = stepinfo(y, t, Ref_rad);
    if isempty(J), Jstr = '   --';
    else,          Jstr = sprintf('%8.3f', J(end));
    end
    fprintf('%-18s : Mp = %5.2f%%   tp = %.3f s   |u|max = %5.2f V   J = %s\n', ...
        name, info.Overshoot, info.PeakTime, max(abs(u)), Jstr);
end
