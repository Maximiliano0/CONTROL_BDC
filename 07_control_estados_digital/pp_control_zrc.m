% =========================================================================
% Control en Espacio de Estados (3x3) - Control de POSICIÓN angular
% del Motor BDC en DOMINIO DISCRETO (Z) - Listo para microcontrolador
% Parámetros: planta real (Ra=11, La=0.008, Kb=0.0014, Je=7.56e-4, Be=1e-5)
% =========================================================================

clear; clc; close all;

disp('=========================================================================');
disp('   Diseño de Control Moderno Digital: Asignación de Polos en Dominio Z   ');
disp('=========================================================================');

%% 1. ESPECIFICACIONES DE DISEÑO Y MUESTREO
% Parámetros configurables por el usuario / ingeniero
Mp = 0.10;         % Sobreimpulso máximo (Ej: 0.40 = 40%)
tp = 1;           % Tiempo pico deseado (s)
Ts = 0.01;         % Tiempo de muestreo: 10 ms
Ref_grados = 10;   % Ángulo objetivo en estado estacionario (Grados)

Ref_rad = deg2rad(Ref_grados); % Conversión obligatoria al SI para la matemática

fprintf('--- 1. Especificaciones ---\n');
fprintf('Sobreimpulso (Mp) : %.0f%%\n', Mp*100);
fprintf('Tiempo pico (tp)  : %.4f s\n', tp);
fprintf('Tiempo de muestreo: %.3f s\n', Ts);
fprintf('Objetivo (Ref)    : %.2f Grados (%.4f Radianes)\n\n', Ref_grados, Ref_rad);

%% 2. MODELO DE LA PLANTA (CONTINUO -> DISCRETO)
% Parámetros físicos
Ra = 11; La = 0.008; Kb = 0.0014; Je = 0.000756; Be = 0.00001;

A = [-Ra/La, -Kb/La,  0 ; 
      Kb/Je, -Be/Je,  0 ;
          0,      1,  0 ]; 

B = [1/La ; 0 ; 0 ];
C = [0, 0, 1];   % ¡Ojo! Solo medimos la posición (Estado 3)
D = 0;

sys_c = ss(A, B, C, D);

% Discretización exacta (ZOH)
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;  
Bd = sys_d.B;  
Cd = sys_d.C;
Dd = sys_d.D;
n_estados = size(Ad, 1);

%% 3. ANÁLISIS DE CONTROLABILIDAD Y OBSERVABILIDAD
disp('--- 2. Análisis de Propiedades del Sistema ---');

Co = ctrb(Ad, Bd);
if rank(Co) == n_estados
    disp('[OK] El sistema es completamente CONTROLABLE.');
else
    error('[ERROR] Sistema NO controlable.');
end

Ob = obsv(Ad, Cd);
if rank(Ob) == n_estados
    disp('[OK] El sistema es completamente OBSERVABLE.');
else
    disp('[ADVERTENCIA] Sistema NO es completamente observable.');
end
disp(' ');

%% 4. CÁLCULO DE POLOS (MAPEO DE S A Z)
zeta = -log(Mp) / sqrt(pi^2 + (log(Mp))^2);
wn = pi / (tp * sqrt(1 - zeta^2));

sigma = zeta * wn;
wd = wn * sqrt(1 - zeta^2);

s1 = -sigma + 1i*wd; 
s2 = -sigma - 1i*wd;
s3 = -10 * sigma; % Polo dominante alejado 

% Mapeo de S a Z (z = exp(s*Ts))
z1 = exp(s1 * Ts);
z2 = exp(s2 * Ts);
z3 = exp(s3 * Ts);
P_z = [z1; z2; z3];

disp('--- 3. Polos Calculados ---');
fprintf('Polo Discreto z1, z2 : %.4f +/- %.4fi\n', real(z1), imag(z1));
fprintf('Polo Discreto z3     : %.4f\n\n', z3);

%% 5. DISEÑO DEL CONTROLADOR DIGITAL
K_z = place(Ad, Bd, P_z);
A_cl_z = Ad - Bd*K_z;
sys_cl_z_sin_ajuste = ss(A_cl_z, Bd, Cd, Dd, Ts);

% Pre-compensador para garantizar error cero en estado estacionario
K_dc = 1 / dcgain(sys_cl_z_sin_ajuste); 

disp('--- 4. Ganancias del Microcontrolador ---');
fprintf('Vector Kz (Realimentación) : [%.4f, %.4f, %.4f]\n', K_z(1), K_z(2), K_z(3));
fprintf('Ganancia K_dc (Pre-comp)   : %.4f\n\n', K_dc);

%% 6. SIMULACIÓN DIGITAL DINÁMICA
% TRUCO DE PROFESOR: El tiempo de simulación se ajusta al 'tp'
% Un sistema de 2do orden se asienta aprox en 4 o 5 veces el tp.
t_end = 4 * tp; 
t_sim = 0:Ts:t_end;
N = length(t_sim);

% El vector de referencia usa radianes internamente
Referencia_rad = Ref_rad * ones(1, N); 

x = zeros(3, N);
y_rad = zeros(1, N);
u = zeros(1, N);

for k = 1:N-1
    y_rad(k) = Cd * x(:, k);
    u(k) = -K_z * x(:, k) + K_dc * Referencia_rad(k);
    x(:, k+1) = Ad * x(:, k) + Bd * u(k);
end
y_rad(N) = Cd * x(:, N);
u(N) = -K_z * x(:, N) + K_dc * Referencia_rad(N);

%% 7. GRÁFICOS Y VALIDACIÓN MEJORADA
% Conversión de la salida matemática a Grados para el usuario
y_grados = rad2deg(y_rad);
Referencia_grados = rad2deg(Referencia_rad);

figure('Name', sprintf('Control Digital Z - Ref: %.1f° | tp: %.1fs', Ref_grados, tp), 'Position', [100, 100, 1000, 450], 'Color', 'w');

% --- Gráfico 1: Posición en GRADOS ---
ax1 = subplot(1,2,1);
plot(t_sim, Referencia_grados, 'k--', 'LineWidth', 2); hold on;
plot(t_sim, y_grados, 'b-', 'LineWidth', 2.5);

% Línea vertical dinámica para tp
xline(tp, 'm:', sprintf('tp = %.2fs', tp), 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5, 'FontSize', 10);

title(sprintf('1. Posición Angular (Objetivo: %.1f°)', Ref_grados), 'FontSize', 12);
xlabel('Tiempo (s)', 'FontWeight', 'bold'); 
ylabel('Posición (Grados °)', 'FontWeight', 'bold');
legend('Referencia', 'Posición del Eje', 'Location', 'SouthEast');
grid on; ax1.GridAlpha = 0.4;
xlim([0, t_end]);
% Ajuste dinámico del eje Y para que el sobreimpulso se vea limpio
ylim([0, Ref_grados * (1 + Mp + 0.15)]); 

% --- Gráfico 2: Voltaje (Esfuerzo de Control) ---
ax2 = subplot(1,2,2);
stairs(t_sim, u, 'r', 'LineWidth', 1.5); hold on;
yline(0, 'k-', 'HandleVisibility','off');

title('2. Esfuerzo de Control (Voltaje V_a)', 'FontSize', 12);
xlabel('Tiempo (s)', 'FontWeight', 'bold'); 
ylabel('Voltaje (V)', 'FontWeight', 'bold');
legend('PWM Discreto (ZOH)', 'Location', 'NorthEast');
grid on; ax2.GridAlpha = 0.4;
xlim([0, t_end]);

% Enlace de ejes X para hacer zoom simultáneo
linkaxes([ax1, ax2], 'x');

%% 8. REPORTE NUMÉRICO FINAL
% Usamos los radianes para que la matemática de stepinfo no falle con escalas
info_real = stepinfo(y_rad, t_sim, Referencia_rad(end));
disp('--- 5. Verificación de Resultados ---');
fprintf('Sobreimpulso Obtenido: %.2f %% (Esperado: %.2f %%)\n', info_real.Overshoot, Mp*100);
fprintf('Tiempo Pico Obtenido : %.4f s (Esperado: %.4f s)\n', info_real.PeakTime, tp);