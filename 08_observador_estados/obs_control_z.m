% =========================================================================
% Observador de Estados (Luenberger) en DOMINIO Z - Motor BDC (3x3)
% Aplicación: control de POSICIÓN angular cuando SOLO se mide theta
% Se estiman en tiempo real ia (corriente) y omega (velocidad) y se
% realimenta el ESTADO ESTIMADO: u = -Kz * x_hat + Kdc * r
% Parámetros: planta real (Ra=11, La=0.008, Kb=0.0014, Je=7.56e-4, Be=1e-5)
% =========================================================================
%
% Estructura del lazo:
%
%   r --(Kdc)--+--> [Planta x[k+1]=Phi*x+Gamma*u] --> y = Cd*x
%              |                                        |
%              v                                        v
%        u = -Kz*xhat                              [Observador]
%                                              xhat[k+1] = Phi*xhat
%                                                 + Gamma*u
%                                                 + L*(y - Cd*xhat)
%
% Principio de separación: Kz y L se diseñan por separado; los polos del
% sistema completo son la unión de los polos de (Phi - Gamma*Kz) y de
% (Phi - L*Cd).
% =========================================================================

clear; clc; close all;

disp('=========================================================================');
disp('   Control con Observador de Estados (Luenberger) - Posición Motor BDC   ');
disp('=========================================================================');

%% 1. PARÁMETROS CONFIGURABLES POR EL USUARIO
Mp           = 0.10;   % Sobreimpulso deseado del CONTROLADOR (10 %)
tp           = 1;      % Tiempo pico deseado del CONTROLADOR (s)
Ts           = 0.01;   % Tiempo de muestreo (s)  -> fs = 100 Hz
Ref_grados   = 10;     % Referencia de posición (Grados)
factor_obs   = 5;      % Polos del observador <factor_obs> veces más rápidos
x0_real      = [0; 0; 0];          % Estado inicial REAL del motor
x0_hat       = [0; 0; deg2rad(2)]; % Estado inicial ESTIMADO (con error)

Ref_rad = deg2rad(Ref_grados);

fprintf('--- 1. Especificaciones ---\n');
fprintf('Mp = %.0f%%   tp = %.3f s   Ts = %.3f s\n', Mp*100, tp, Ts);
fprintf('Referencia: %.1f° (%.4f rad)\n', Ref_grados, Ref_rad);
fprintf('Polos del observador: %dx más rápidos que el controlador\n\n', factor_obs);

%% 2. MODELO DE LA PLANTA (CONTINUO -> DISCRETO ZOH)
Ra = 11; La = 0.008; Kb = 0.0014; Je = 0.000756; Be = 0.00001;

A = [-Ra/La, -Kb/La, 0 ;
      Kb/Je, -Be/Je, 0 ;
          0,      1, 0 ];
B = [1/La ; 0 ; 0];
C = [0, 0, 1];   % Solo medimos posición theta (encoder)
D = 0;

sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts, 'zoh');

Phi   = sys_d.A;
Gamma = sys_d.B;
Cd    = sys_d.C;
Dd    = sys_d.D;
n     = size(Phi,1);

%% 3. CONTROLABILIDAD Y OBSERVABILIDAD
disp('--- 2. Análisis del Par (Phi, Cd) ---');
Co = ctrb(Phi, Gamma);
Ob = obsv(Phi, Cd);
fprintf('rank(Controlabilidad) = %d  (n = %d)\n', rank(Co), n);
fprintf('rank(Observabilidad)  = %d  (n = %d)\n', rank(Ob), n);
if rank(Co) < n, error('Sistema NO controlable.'); end
if rank(Ob) < n, error('Sistema NO observable: imposible estimar estados.'); end
disp('[OK] Sistema controlable Y observable.');
disp(' ');

%% 4. POLOS DESEADOS DEL CONTROLADOR (mapeo s -> z)
% Eq. (3.x): z_i = exp(s_i * Ts) - mapeo exacto de polos por ZOH
zeta  = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
wn    = pi / (tp * sqrt(1 - zeta^2));
sigma = zeta * wn;
wd    = wn * sqrt(1 - zeta^2);

% Tercer polo 10x más rápido para forzar que los polos dominantes sean el par complejo
s_ctrl = [-sigma + 1i*wd, -sigma - 1i*wd, -10*sigma];
P_ctrl = exp(s_ctrl * Ts);

% Asignación de polos en lazo cerrado: u = -Kz*x  ->  (Phi - Gamma*Kz)
Kz = place(Phi, Gamma, P_ctrl);

% Pre-compensador para ganancia DC unitaria (Eq. 7.x del cap. 07)
% Garantiza que ante ref. constante r, la salida en régimen alcance r.
A_cl = Phi - Gamma*Kz;
Kdc  = 1 / (Cd * ((eye(n) - A_cl) \ Gamma));

fprintf('--- 3. Ganancias del Controlador ---\n');
fprintf('Kz   = [%.4f, %.4f, %.4f]\n', Kz(1), Kz(2), Kz(3));
fprintf('Kdc  = %.4f\n\n', Kdc);

%% 5. POLOS DESEADOS DEL OBSERVADOR (más rápidos que el controlador)
s_obs = factor_obs * s_ctrl;     % mismo patrón, pero "factor_obs" veces más rápido
P_obs = exp(s_obs * Ts);

% Diseño por dualidad: L = place(Phi', Cd', P_obs)'
L = place(Phi', Cd', P_obs).';

fprintf('--- 4. Ganancia del Observador ---\n');
fprintf('L = [%.4f; %.4f; %.4f]\n', L(1), L(2), L(3));
fprintf('Polos del observador: '); disp(P_obs.');

%% 6. VERIFICACIÓN DEL PRINCIPIO DE SEPARACIÓN
% Sistema aumentado de orden 2n con estados [x ; e],  e = x - xhat:
%   [x[k+1]]   [Phi - Gamma*Kz   Gamma*Kz ] [x]
%   [e[k+1]] = [     0           Phi - L*Cd] [e]
% Polos del sistema completo = polos controlador U polos observador
Aaug = [Phi - Gamma*Kz, Gamma*Kz ; zeros(n), Phi - L*Cd];
fprintf('Polos del sistema completo (aumentado):\n');
disp(eig(Aaug));
fprintf('Deben coincidir con la unión de P_ctrl y P_obs.\n\n');

%% 7. SIMULACIÓN DINÁMICA NO LINEAL (planta + observador + control)
t_end = 6 * tp;
t_sim = 0:Ts:t_end;
N     = length(t_sim);

Ref = Ref_rad * ones(1, N);

% --- Caso A: control IDEAL con estado REAL (referencia) ---
x_ideal = zeros(n, N); x_ideal(:,1) = x0_real;
u_ideal = zeros(1, N);

% --- Caso B: control con OBSERVADOR (lo que correrá en el micro) ---
x_real = zeros(n, N); x_real(:,1) = x0_real;
xhat   = zeros(n, N); xhat(:,1)   = x0_hat;
u_obs  = zeros(1, N);
y_meas = zeros(1, N);

for k = 1:N-1
    % --- (A) ideal: realimentamos el estado real ---
    u_ideal(k)      = -Kz * x_ideal(:,k) + Kdc * Ref(k);
    x_ideal(:,k+1)  = Phi * x_ideal(:,k) + Gamma * u_ideal(k);

    % --- (B) observador: realimentamos el estado estimado ---
    y_meas(k)       = Cd * x_real(:,k);                 % única medición: posición (encoder)
    u_obs(k)        = -Kz * xhat(:,k) + Kdc * Ref(k);   % ley de control con xhat
    % Planta real evoluciona con la entrada aplicada:
    x_real(:,k+1)   = Phi * x_real(:,k)  + Gamma * u_obs(k);
    % Observador (predictor): Eq. (8.x) del cap. 08
    %   xhat[k+1] = Phi*xhat[k] + Gamma*u[k] + L*(y[k] - Cd*xhat[k])
    % El término (y - Cd*xhat) es la INNOVACIÓN; L pondera cuánto
    % corregir cada estado estimado a partir de esa innovación.
    xhat(:,k+1)     = Phi * xhat(:,k)    + Gamma * u_obs(k) ...
                    + L * (y_meas(k) - Cd * xhat(:,k));
end
y_meas(N)  = Cd * x_real(:,N);
u_ideal(N) = u_ideal(N-1);
u_obs(N)   = u_obs(N-1);

% Errores de estimación
err_ia = x_real(1,:) - xhat(1,:);
err_we = x_real(2,:) - xhat(2,:);
err_th = x_real(3,:) - xhat(3,:);

%% 8. GRÁFICOS
figure('Name', sprintf('Observador Luenberger (Ref %.1f°, tp %.2fs)', Ref_grados, tp), ...
       'Position', [60, 60, 1100, 800], 'Color', 'w');

% 8.1 Posición: ideal vs observador vs referencia
ax1 = subplot(3,2,1);
plot(t_sim, rad2deg(Ref),           'k--', 'LineWidth', 2); hold on;
plot(t_sim, rad2deg(x_ideal(3,:)),  'Color',[0 0.45 0.74], 'LineWidth', 2);
plot(t_sim, rad2deg(x_real(3,:)),   'Color',[0.85 0.33 0.10], 'LineWidth', 2);
title('1. Posición \theta — control ideal vs. con observador');
xlabel('Tiempo (s)'); ylabel('\theta (°)');
legend('Referencia','Estado real (ideal)','Con observador','Location','SouthEast');
grid on; ax1.GridAlpha = 0.3;

% 8.2 Esfuerzo de control
ax2 = subplot(3,2,2);
stairs(t_sim, u_ideal, 'Color',[0 0.45 0.74], 'LineWidth', 1.5); hold on;
stairs(t_sim, u_obs,   'Color',[0.85 0.33 0.10], 'LineWidth', 1.5);
yline(0,'k-','HandleVisibility','off');
title('2. Esfuerzo de control u[k] (V)');
xlabel('Tiempo (s)'); ylabel('V_a (V)');
legend('Ideal','Con observador','Location','NorthEast');
grid on; ax2.GridAlpha = 0.3;

% 8.3 Corriente: real vs estimada
ax3 = subplot(3,2,3);
plot(t_sim, x_real(1,:), 'Color',[0.85 0.33 0.10], 'LineWidth', 2); hold on;
plot(t_sim, xhat(1,:),   'Color',[0 0.45 0.74], 'LineWidth', 1.5, 'LineStyle','--');
title('3. Corriente i_a: real vs. estimada');
xlabel('Tiempo (s)'); ylabel('i_a (A)');
legend('Real','Estimada','Location','best');
grid on; ax3.GridAlpha = 0.3;

% 8.4 Velocidad: real vs estimada
ax4 = subplot(3,2,4);
plot(t_sim, x_real(2,:), 'Color',[0.85 0.33 0.10], 'LineWidth', 2); hold on;
plot(t_sim, xhat(2,:),   'Color',[0 0.45 0.74], 'LineWidth', 1.5, 'LineStyle','--');
title('4. Velocidad \omega: real vs. estimada');
xlabel('Tiempo (s)'); ylabel('\omega (rad/s)');
legend('Real','Estimada','Location','best');
grid on; ax4.GridAlpha = 0.3;

% 8.5 Error de estimación (todos los estados)
ax5 = subplot(3,2,5);
plot(t_sim, err_ia,            'LineWidth', 1.5); hold on;
plot(t_sim, err_we,            'LineWidth', 1.5);
plot(t_sim, rad2deg(err_th),   'LineWidth', 1.5);
yline(0,'k-','HandleVisibility','off');
title('5. Error de estimación e = x - \^x');
xlabel('Tiempo (s)'); ylabel('Error');
legend('e_{i_a} (A)','e_{\omega} (rad/s)','e_{\theta} (°)','Location','best');
grid on; ax5.GridAlpha = 0.3;

% 8.6 Polos del sistema completo
ax6 = subplot(3,2,6);
theta = linspace(0,2*pi,200);
plot(cos(theta), sin(theta), 'k-', 'LineWidth', 1); hold on; axis equal;
plot(real(P_ctrl), imag(P_ctrl), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
plot(real(P_obs),  imag(P_obs),  'rs', 'MarkerSize', 10, 'LineWidth', 2);
xline(0,'k:','HandleVisibility','off'); yline(0,'k:','HandleVisibility','off');
title('6. Polos en plano Z (separación)');
xlabel('Re'); ylabel('Im');
legend('|z| = 1','Controlador','Observador','Location','SouthWest');
grid on; ax6.GridAlpha = 0.3;
xlim([-1.2 1.2]); ylim([-1.2 1.2]);

linkaxes([ax1, ax2, ax3, ax4, ax5], 'x');
xlim(ax1, [0, t_end]);

%% 9. REPORTE NUMÉRICO
info_ideal = stepinfo(x_ideal(3,:), t_sim, Ref_rad);
info_obs   = stepinfo(x_real(3,:),  t_sim, Ref_rad);
disp('--- 5. Verificación de Resultados ---');
fprintf('IDEAL       -> Mp = %.2f%%   tp = %.4f s\n', info_ideal.Overshoot, info_ideal.PeakTime);
fprintf('OBSERVADOR  -> Mp = %.2f%%   tp = %.4f s\n', info_obs.Overshoot,   info_obs.PeakTime);
fprintf('Esperado    -> Mp = %.2f%%   tp = %.4f s\n', Mp*100, tp);
