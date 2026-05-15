%% ========================================================================
% SISTEMA EN LAZO CERRADO DIGITAL (Retroalimentación)
% ========================================================================
clc; clear; close all;

%% 1. DEFINICIÓN DEL SISTEMA EN LAZO ABIERTO
Fs = 1000;      % Frecuencia de muestreo: 1 kHz
Ts = 1 / Fs;    

num = [1, -0.5]; 
den = [1, -1.5, 0.7];
sys_ol = tf(num, den, Ts); % Sistema Lazo Abierto (Open Loop)

%% 2. CERRAR EL LAZO (Feedback digital)
% Simulamos un controlador proporcional de ganancia 1 (K=1)
sys_cl = feedback(sys_ol, 1);

disp('Sistema Lazo Abierto G(z):'); display(sys_ol);
disp('Sistema Lazo Cerrado G_cl(z) = G/(1+G):'); display(sys_cl);

%% 3. ESTABILIDAD EN LAZO CERRADO
poles_cl = pole(sys_cl);

fprintf('--- Estabilidad Lazo Cerrado ---\n');
for i = 1:length(poles_cl)
    fprintf('Polo CL %d: Magnitud = %.4f\n', i, abs(poles_cl(i)));
end

if all(abs(poles_cl) < 1)
    fprintf('>> SISTEMA EN LAZO CERRADO ESTABLE.\n');
else
    fprintf('>> SISTEMA EN LAZO CERRADO INESTABLE.\n');
end

%% 4. COMPARATIVA VISUAL (Abierto vs Cerrado)
figure('Name', 'Comparación: Abierto vs Cerrado', 'Position', [100, 100, 900, 400]);

% Gráfico 1: Movimiento de Polos (Usando pzmap en lugar de zplane)
subplot(1,2,1);
pzmap(sys_ol, 'b', sys_cl, 'r'); 
title('Movimiento de Polos (Plano Z)');
legend('Lazo Abierto', 'Lazo Cerrado', 'Location', 'SouthWest');
grid on;

% Gráfico 2: Respuesta al Escalón (Comportamiento real de DAC)
subplot(1,2,2);

% 1. Extraemos los datos numéricos de la simulación
[y_ol, t_ol] = step(sys_ol);
[y_cl, t_cl] = step(sys_cl);

% 2. Graficamos usando 'stairs' (escaleras)
stairs(t_ol, y_ol, 'b--', 'LineWidth', 1.2); hold on;
stairs(t_cl, y_cl, 'r-', 'LineWidth', 1.5);

title('Seguimiento de Referencia (Escalón)');
legend('Lazo Abierto', 'Lazo Cerrado', 'Location', 'SouthEast');
xlabel('Tiempo (s)'); ylabel('Amplitud');
grid on; 