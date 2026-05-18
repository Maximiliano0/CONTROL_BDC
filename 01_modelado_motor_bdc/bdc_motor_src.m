% =========================================================================
% Cap. 01 — MODELADO DEL MOTOR BDC (BRUSHED DC MOTOR)
% -------------------------------------------------------------------------
% Propósito  : Obtener la función de transferencia y la representación en
%              espacio de estados (modelo 2x2: corriente y velocidad) de
%              un motor BDC y visualizar su comportamiento dinámico.
% Aplicación : Motor BDC, salida = velocidad angular ω(t).
% Parámetros : Didácticos (Ra=0.5, La=0.5, K=0.01, Je=0.01, Be=0.1).
% Muestreo   : — (modelo continuo).
% Entradas   : Ninguna (los parámetros se modifican en el código).
% Salidas    : Reporte en consola (G(s), A,B,C,D, polos y ceros) y figura
%              con respuesta al escalón, Bode, polos/ceros y root locus.
% Doc        : docs/01_modelado_motor_bdc.md
% =========================================================================
clear; clc; close all;

% --- Parámetros del Motor BDC (didácticos) ---
Ra = 0.5;       % Resistencia de armadura (Ohms)
La = 0.5;       % Inductancia (H)
K  = 0.01;      % Constante de torque (Nm/A)
Je = 0.01;      % Inercia del rotor (kg*m^2)
Be = 0.1;       % Fricción viscosa (N*m*s)

% --- Modelado Matemático ---
num = K;
den = [(Je*La), (Je*Ra + Be*La), (Be*Ra + K^2)];
G_tf = tf(num, den);

% --- Representación en Espacio de Estados ---
A = [-Ra/La, -K/La; K/Je, -Be/Je];
B = [1/La; 0];
C = [0, 1];
D = 0; 
sys_ss = ss(A, B, C, D);

% --- Visualización en Terminal (Reporte de Datos) ---
fprintf('==================================================\n');
fprintf('       REPORTE DE ANÁLISIS: MOTOR DC              \n');
fprintf('==================================================\n');

fprintf('\n1. FUNCIÓN DE TRANSFERENCIA G(s) = Omega(s)/V(s):\n');
display(G_tf);

fprintf('--------------------------------------------------\n');
fprintf('2. MATRICES DE ESTADO (Espacio de Estados):\n\n');

fprintf('Matriz A (Dinámica del sistema):\n');
disp(A);
fprintf('Matriz B (Entrada):\n');
disp(B);
fprintf('Matriz C (Salida):\n');
disp(C);
fprintf('Matriz D (Transmisión directa):\n');
disp(D);

fprintf('--------------------------------------------------\n');
fprintf('3. POLOS Y CEROS:\n');
fprintf('Polos: \n'); disp(pole(G_tf));
fprintf('Ceros: \n'); disp(zero(G_tf));
fprintf('==================================================\n');

% --- Visualización Gráfica Optimizada ---
figure('Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8], 'Name', 'Análisis de Control - Motor DC');

% 1. Respuesta al Escalón
subplot(2,2,1);
[y, t] = step(G_tf);
plot(t, y, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
grid on;
title('\bfRespuesta al Escalón Unitario', 'FontSize', 12);
xlabel('Tiempo (s)'); ylabel('Velocidad (\omega)');

% Marcadores de tiempo de asentamiento
info = stepinfo(G_tf);
y_final = dcgain(G_tf); 
hold on;
line([info.SettlingTime info.SettlingTime], [0 y_final], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.2);
text(info.SettlingTime, y_final/2, sprintf('  t_s = %.2f s', info.SettlingTime), 'Color', 'r', 'FontWeight', 'bold');
hold off;

% 2. Mapa de Polos y Ceros
subplot(2,2,2);
pzmap(G_tf);
grid on;
h = findobj(gca, 'Type', 'Line');
set(h, 'LineWidth', 2, 'MarkerSize', 10);
title('\bfMapa de Polos y Ceros', 'FontSize', 12);

% 3. Diagrama de Bode con Márgenes de Estabilidad
subplot(2,2,3);
margin(G_tf); 
grid on;
set(findall(gca, 'Type', 'line'), 'LineWidth', 1.5);

% 4. Lugar Geométrico de las Raíces (Root Locus)
subplot(2,2,4);
rlocus(G_tf);
grid on;
title('\bfLugar Geométrico de las Raíces', 'FontSize', 12);
set(findall(gca, 'Type', 'line'), 'LineWidth', 1.5);

% Título General
sgtitle('Análisis Dinámico del Motor DC', 'FontSize', 16, 'FontWeight', 'bold');

% --- Segunda Figura: Evolución de Estados ---
figure('Units', 'normalized', 'Position', [0.2, 0.2, 0.5, 0.5], 'Name', 'Estados del Sistema', 'Color', 'w');
[y_ss, t_ss, x] = step(sys_ss);
plot(t_ss, x(:,1), 'r', 'LineWidth', 2); hold on;
plot(t_ss, x(:,2), 'b', 'LineWidth', 2);
grid on;
legend({'i_a (Corriente de Armadura)', '\omega (Velocidad Angular)'}, 'Location', 'best');
title('\bfEvolución Temporal de las Variables de Estado', 'FontSize', 12);
xlabel('Tiempo (s)');
ylabel('Amplitud (A o rad/s)');