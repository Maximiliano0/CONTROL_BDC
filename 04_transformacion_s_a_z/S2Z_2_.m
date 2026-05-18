% =========================================================================
% Cap. 04 — DOMINIOS S↔Z: mapeo, discretización, ROC y respuesta en frec.
% -------------------------------------------------------------------------
% Propósito  : Cinco figuras pedagógicas que ilustran los conceptos
%              fundamentales del paso del dominio continuo (s) al
%              discreto (z) mediante z = exp(s·Ts).
% Aplicación : Sistema genérico de 2º orden subamortiguado.
%              NO representa la planta del motor (ejemplo didáctico).
% Parámetros : wn = 5 rad/s, ζ = 0.3, Ts_base = 0.1 s.
% Muestreo   : Varía por figura (Ts_base = 0.1 s como referencia).
% Entradas   : Ninguna.
% Salidas    : Cinco figuras pedagógicas (ver lista abajo).
% Doc        : docs/04_transformacion_s_a_z.md
% -------------------------------------------------------------------------
% FIGURAS GENERADAS:
%   1. Correspondencia S↔Z mediante z = exp(s·Ts).
%   2. G(s) continuo vs. G(z) discreto (efecto visible del ZOH).
%   3. Migración de polos en z al variar el tiempo de muestreo Ts.
%   4. Región de Convergencia (ROC) y fronteras de estabilidad.
%   5. Periodicidad del espectro discreto (aliasing alrededor de fs/2).
% =========================================================================

clear; clc; close all;

%% SISTEMA DE EJEMPLO
% Sistema de 2do orden subamortiguado: oscila al ser excitado, lo que hace
% visibles los efectos del muestreo sobre la dinámica transitoria.
wn   = 5;       % Frecuencia natural [rad/s]
zeta = 0.3;     % Coeficiente de amortiguamiento (subamortiguado: zeta < 1)
G_s  = tf(wn^2, [1, 2*zeta*wn, wn^2]);  % G(s) = 25 / (s² + 3s + 25)

% Tiempo de muestreo base para los análisis temporales y de mapeo
Ts_base = 0.1;

%% 1. CORRESPONDENCIA PLANO S <-> PLANO Z   (z = exp(s·Ts))
% Cada punto del plano s se mapea al plano z mediante la exponencial compleja.
% Visualizamos las dos familias de líneas características:
%   sigma constante (decaimiento)  →  circunferencias concéntricas en z
%   omega constante (frecuencia)   →  rayos radiales en z
%   eje imaginario (sigma = 0)     →  círculo unitario en z  (frontera de estabilidad)
figure('Name', '1. Correspondencia Plano S - Plano Z', 'NumberTitle', 'off', 'Position', [50, 100, 1000, 400]);

% Plano S
subplot(1,2,1); hold on; grid on;
title('Plano S: líneas de \sigma y \omega constantes');
xlabel('Real (\sigma)'); ylabel('Imaginario (j\omega)');
xlim([-10, 2]); ylim([-15, 15]);
xline(0, 'k', 'LineWidth', 2);  % Eje imaginario = frontera de estabilidad en s

% Plano Z
subplot(1,2,2); hold on; grid on; axis equal;
title('Plano Z: imagen del plano S  (z = e^{sT_s})');
xlabel('Real'); ylabel('Imaginario');
xlim([-1.5, 1.5]); ylim([-1.5, 1.5]);
% Círculo unitario = frontera de estabilidad en z
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k', 'LineWidth', 2);

% Líneas de sigma constante → circunferencias de radio exp(sigma·Ts)
% sigma < 0  →  radio < 1  →  interior del círculo unitario (estable)
sigmas      = [-1, -3, -5, -8];
omega_range = linspace(-15, 15, 200);
colors      = lines(length(sigmas));
for i = 1:length(sigmas)
    s_line = sigmas(i) + 1j*omega_range;
    z_line = exp(s_line * Ts_base);
    subplot(1,2,1); plot(real(s_line), imag(s_line), 'Color', colors(i,:), 'LineWidth', 1.5);
    subplot(1,2,2); plot(real(z_line), imag(z_line), 'Color', colors(i,:), 'LineWidth', 1.5);
end

% Líneas de omega constante → rayos con ángulo omega·Ts en z
% omega = fs/2  corresponde al punto z = -1 (frecuencia de Nyquist)
omegas      = [3, 6, 9, 12];
sigma_range = linspace(-10, 0, 100);
for i = 1:length(omegas)
    s_line1 = sigma_range + 1j*omegas(i);
    s_line2 = sigma_range - 1j*omegas(i);
    z_line1 = exp(s_line1 * Ts_base);
    z_line2 = exp(s_line2 * Ts_base);
    subplot(1,2,1);
    plot(real(s_line1), imag(s_line1), 'k--', 'LineWidth', 1);
    plot(real(s_line2), imag(s_line2), 'k--', 'LineWidth', 1);
    subplot(1,2,2);
    plot(real(z_line1), imag(z_line1), 'k--', 'LineWidth', 1);
    plot(real(z_line2), imag(z_line2), 'k--', 'LineWidth', 1);
end
legend('Círculo unitario (s=j\omega)', 'Mapeo \sigma cte.', 'Location', 'best');

%% 2. G(s) CONTINUO vs. G(z) DISCRETO (efecto del ZOH)
% Discretizamos con ZOH: modela el comportamiento real de un DAC en un
% microcontrolador, que mantiene constante su salida entre muestras.
% Comparamos tres representaciones:
%   Curva azul   – G(s) continuo (referencia suave)
%   Escalones rojos – G(z) ZOH tal como sale del DAC ("escalera")
%   Puntos negros   – Muestras ideales G*(s) (señal discreta pura)
figure('Name', '2. G(s) vs G*(s) vs G(z)', 'NumberTitle', 'off', 'Position', [100, 150, 800, 450]);

G_z = c2d(G_s, Ts_base, 'zoh');

t_cont = 0:0.005:3;
t_disc = 0:Ts_base:3;

[y_c, ~] = step(G_s, t_cont);
[y_d, ~] = step(G_z, t_disc);

hold on; grid on;
plot(t_cont, y_c, 'b', 'LineWidth', 2, 'DisplayName', 'Continuo G(s)');
stairs(t_disc, y_d, 'r', 'LineWidth', 1.5, 'DisplayName', 'Discreto G(z) (escalera ZOH)');
stem(t_disc, y_d, 'k', 'Filled', 'DisplayName', 'Muestras ideales G*(s)');
title(['Respuesta al Escalón – Continuo vs. Discreto  (Ts = ', num2str(Ts_base), ' s)']);
xlabel('Tiempo [s]'); ylabel('Amplitud');
legend('Location', 'Southeast');

%% 3. MIGRACIÓN DE POLOS DE G(z) AL VARIAR Ts
% Para un mismo G(s), si Ts crece (muestreo más lento) los polos discretos
% se alejan del origen y se acercan al círculo unitario. Si Ts es demasiado
% grande los polos salen del círculo → sistema discreto inestable aunque
% el continuo sea estable. Esto ilustra por qué Ts no se puede elegir libre.
figure('Name', '3. Migración de polos al variar Ts', 'NumberTitle', 'off', 'Position', [150, 200, 600, 600]);
hold on; grid on; axis equal;
zgrid;  % Grilla del plano z con curvas de amortiguamiento y frecuencia constantes
title('Migración de polos en Z al aumentar T_s');

% Barrido de Ts: de muy rápido (0.01 s) a muy lento (0.4 s)
Ts_array = [0.01, 0.05, 0.1, 0.2, 0.3, 0.4];
colors   = parula(length(Ts_array));

for i = 1:length(Ts_array)
    Ts_current  = Ts_array(i);
    G_z_current = c2d(G_s, Ts_current, 'zoh');
    p_z = pole(G_z_current);

    plot(real(p_z), imag(p_z), 'x', 'MarkerSize', 10, 'LineWidth', 2, ...
         'Color', colors(i,:), 'DisplayName', ['Ts = ', num2str(Ts_current), ' s']);
end
legend('Location', 'southwest');

%% 4. REGIÓN DE CONVERGENCIA (ROC) Y FRONTERAS DE ESTABILIDAD
% Resumen visual de la "regla de oro" del control digital:
%   Interior del círculo unitario  (|z| < 1)  →  ESTABLE
%   Exterior del círculo unitario  (|z| > 1)  →  INESTABLE
%   Sobre el círculo unitario      (|z| = 1)  →  oscilación sostenida
figure('Name', '4. ROC y Estabilidad en el Dominio Z', 'NumberTitle', 'off', 'Position', [200, 250, 600, 600]);
hold on; grid on; axis equal;
title('Región de Convergencia (ROC) y Estabilidad en Z');
xlabel('Real'); ylabel('Imaginario');
xlim([-2, 2]); ylim([-2, 2]);

% Zona estable (verde): interior del círculo unitario
fill(cos(theta), sin(theta), [0.8 1 0.8], 'EdgeColor', 'k', 'LineWidth', 2, ...
     'DisplayName', 'Región Estable (|z| < 1)');

% Zona inestable (rosa): exterior, acotada visualmente a radio 3
theta_outer = [theta, fliplr(theta)];
r_outer     = [ones(1,100)*3, ones(1,100)];
x_outer     = r_outer .* cos(theta_outer);
y_outer     = r_outer .* sin(theta_outer);
fill(x_outer, y_outer, [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, ...
     'DisplayName', 'Región Inestable (|z| > 1)');

% Ejemplos pedagógicos de polos con comportamientos típicos
plot( 0.8,  0.4, 'bx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Estable: oscilación decreciente');
plot( 0.95, 0.0, 'gx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Estable: decaimiento lento');
plot( 1.1,  0.5, 'rx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Inestable: oscilación creciente');
plot(-1.2,  0.0, 'kx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Inestable: ringing creciente');

legend('Location', 'northeast');

%% 5. RESPUESTA EN FRECUENCIA Y PERIODICIDAD DEL ESPECTRO DISCRETO
% Los sistemas discretos tienen espectro PERIÓDICO con periodo fs:
%   H(e^{j·w·Ts}) se repite cada vez que w avanza 2π·fs.
% Por eso el rango "útil" de frecuencias es [0, fs/2] (Nyquist).
% Graficamos hasta 3·fs para que la periodicidad sea evidente.
figure('Name', '5. Respuesta en Frecuencia y Periodicidad', 'NumberTitle', 'off', 'Position', [250, 300, 900, 600]);

% Parámetros del ejemplo de filtros
fs      = 10;        % Frecuencia de muestreo [Hz]
Ts_bode = 1/fs;
fc      = 2;         % Frecuencia de corte [Hz]
wc      = 2*pi*fc;

% Filtros continuos de primer orden
LPF_s = tf(wc,     [1, wc]);  % Pasa-bajos:  wc / (s + wc)
HPF_s = tf([1, 0], [1, wc]);  % Pasa-altos:  s  / (s + wc)

% Discretización con Tustin (bilineal): mapeo biyectivo entre semiplano
% izquierdo e interior del círculo unitario → preserva estabilidad siempre.
LPF_z = c2d(LPF_s, Ts_bode, 'tustin');
HPF_z = c2d(HPF_s, Ts_bode, 'tustin');

% Vector de frecuencias extendido hasta 3·fs para ver las réplicas espectrales.
% La función bode() de MATLAB corta en Nyquist (fs/2); usamos freqresp() manual.
f_vec = linspace(0, 3*fs, 1000);  % 0 a 30 Hz
w_vec = 2*pi*f_vec;

H_LPF = squeeze(freqresp(LPF_z, w_vec));
H_HPF = squeeze(freqresp(HPF_z, w_vec));

Mag_LPF_dB = 20*log10(abs(H_LPF));
Mag_HPF_dB = 20*log10(abs(H_HPF));

% Pasa-bajos: la magnitud se repite periódicamente cada fs
subplot(2,1,1); hold on; grid on;
plot(f_vec, Mag_LPF_dB, 'b', 'LineWidth', 2);
title('Filtro Digital Pasa-Bajos – nótese la periodicidad cada f_s');
ylabel('Magnitud [dB]');
ylim([-40 5]);
% Marcamos múltiplos de fs/2 para identificar Nyquist y réplicas
for k = 1:3
    xline(k*fs/2, 'k--', ['k·f_s/2 = ', num2str(k*fs/2), ' Hz'], 'LabelOrientation', 'horizontal');
end

% Pasa-altos: mismo fenómeno de periodicidad
subplot(2,1,2); hold on; grid on;
plot(f_vec, Mag_HPF_dB, 'r', 'LineWidth', 2);
title('Filtro Digital Pasa-Altos – mismo efecto periódico');
xlabel('Frecuencia [Hz]'); ylabel('Magnitud [dB]');
ylim([-40 5]);
for k = 1:3
    xline(k*fs/2, 'k--', ['k·f_s/2 = ', num2str(k*fs/2), ' Hz'], 'LabelOrientation', 'horizontal');
end

disp('Ejecución completa. Revisar las 5 figuras generadas.');