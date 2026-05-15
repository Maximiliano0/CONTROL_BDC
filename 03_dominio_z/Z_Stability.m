%% ========================================================================
% ANÁLISIS DE ESTABILIDAD Y RESPUESTA (Lazo Abierto en 'z')
% ========================================================================
clc; clear; close all;

%% 1. PARÁMETROS DEL SISTEMA DIGITAL
Fs = 100;       % Frecuencia de muestreo: 100 Hz
Ts = 1 / Fs;    % Tiempo de muestreo: 10 ms

% Numerador y Denominador de nuestro sistema digital G(z)
% G(z) = (z - 0.5) / (z^2 - 1.5z + 0.7)
num = [1, -0.5]; 
den = [1, -1.5, 0.7];

% Creación del objeto Transfer Function Discreto
sys = tf(num, den, Ts); 
disp('Sistema en Lazo Abierto G(z):');
display(sys);

%% 2. ANÁLISIS DE ESTABILIDAD (La Regla de Oro del Círculo Unitario)
polos = pole(sys);
ceros = zero(sys);

% Calculamos la magnitud de los polos. 
% Si TODOS los polos tienen magnitud < 1, es estable.
isStable = all(abs(polos) < 1);

fprintf('--- Diagnóstico de Estabilidad ---\n');
for i = 1:length(polos)
    fprintf('Polo %d: %.4f + %.4fi | Magnitud: %.4f\n', i, real(polos(i)), imag(polos(i)), abs(polos(i)));
end

if isStable
    fprintf('>> EL SISTEMA ES ESTABLE (Todos los polos dentro del círculo unitario).\n\n');
else
    fprintf('>> PELIGRO: EL SISTEMA ES INESTABLE.\n\n');
end

%% 3. GRÁFICOS DE SISTEMA (Plano Z, Escalón y Bode Digital)
figure('Name', 'Análisis de Lazo Abierto Digital', 'Position', [100, 100, 1200, 400]);

% Gráfico 1: Plano Z
subplot(1,3,1);
zplane(num, den);
title('Plano Z: Círculo Unitario');
grid on;

% Gráfico 2: Respuesta al Escalón (Temporal)
subplot(1,3,2);
% 1. Extraemos los vectores de tiempo (t) y amplitud (y) de la simulación
[y, t] = step(sys); 

% 2. Dibujamos usando 'stairs' (escalera) que es la representación 
% REAL de un sistema digital retenido (ZOH) en un microcontrolador.
stairs(t, y, 'b', 'LineWidth', 1.5); 
title('Respuesta al Escalón (Digital)');
xlabel('Tiempo (s)'); ylabel('Amplitud');
grid on;

% Gráfico 3: Respuesta en Frecuencia (Espectro Digital)
% En lugar de freqz directo, usamos 'bode' que es más estándar en control,
% o mantenemos freqz pero con mejor estética.
subplot(1,3,3);
N = 1024; % Puntos de la FFT
[h, w] = freqz(num, den, N, Fs);
plot(w, 20*log10(abs(h)), 'k', 'LineWidth', 1.5);
title('Magnitud (Espectro Frecuencial)');
xlabel('Frecuencia (Hz)'); ylabel('Magnitud (dB)');
xlim([0 Fs/2]); % Según Nyquist, solo vemos hasta Fs/2
grid on;