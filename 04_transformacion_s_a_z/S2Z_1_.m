% =========================================================================
% Cap. 04 — TRANSFORMACIÓN S → Z: comparación de métodos (ZOH e Impulse)
% -------------------------------------------------------------------------
% Propósito  : Tomar un sistema continuo G(s) y obtener su equivalente
%              discreto G(z) por dos métodos clásicos (ZOH e Impulse
%              Invariant), comparando sus respuestas al escalón e impulso.
% Aplicación : Sistema genérico G(s) = 1 / (2s² + s + 5).
%              NO representa la planta del motor (ejemplo didáctico).
% Parámetros : Coeficientes fijos del polinomio en s.
% Muestreo   : Ts = 1 ms (Fs = 1 kHz).
% Entradas   : Ninguna.
% Salidas    : G_s, G_z_zoh, G_z_imp y Ts exportados al workspace para
%              usar en S2Z_1_Sim.slx; figura comparativa de respuestas.
% Doc        : docs/04_transformacion_s_a_z.md
% -------------------------------------------------------------------------
% FLUJO DE USO:
%   1. Ejecutar este script  →  calcula G(s), G_z_zoh y G_z_imp.
%   2. Abrir S2Z_1_Sim.slx   →  usa esas variables para mostrar el efecto
%      del bloque ZOH (retención de orden cero) sobre la señal de control.
% =========================================================================

clear; clc; close all;

%% 1. UNIDADES Y PARÁMETROS DE MUESTREO
% Definimos escalas simbólicas para que los valores numéricos sean
% legibles directamente con sus unidades físicas.
seg  = 1;
mseg = seg * (10^-3);
Hz   = 1/seg;
kHz  = Hz * (10^3);

Fs = 1 * kHz;   % Frecuencia de muestreo: 1 kHz  (muestreo muy rápido)
Ts = 1/Fs;      % Periodo de muestreo: 1 ms
N  = 1024;      % Reservado para análisis espectral (FFT)

%% 2. SISTEMA CONTINUO DE EJEMPLO
% G(s) = 1 / (2s² + s + 5) — sistema genérico de 2do orden subamortiguado.
% NO representa al motor BDC; su único propósito es ilustrar con claridad
% las diferencias entre los métodos de discretización.
s   = tf('s');
G_s = 1 / (2*s^2 + s + 5);

% Extraemos coeficientes como vectores fila: útil para depuración o para
% escribir manualmente la ecuación en diferencias en el microcontrolador.
[Gn, Gd] = tfdata(G_s, 'v');

%% 3. DISCRETIZACIÓN CON DOS MÉTODOS
% c2d(G, Ts, método) convierte la FT continua al dominio discreto.

% — ZOH (Zero-Order Hold) —
%   Modela exactamente lo que hace el DAC de un microcontrolador: mantiene
%   el valor de la señal de control constante entre instantes de muestreo.
%   Es el método estándar en control digital embebido.
G_z_zoh = c2d(G_s, Ts, 'zoh');

% — Impulse Invariant —
%   Diseñado para igualar la respuesta al impulso muestreada del sistema
%   continuo. Se usa más en procesamiento de señales que en control.
G_z_imp = c2d(G_s, Ts, 'impulse');

% Coeficientes de G_z_zoh como vectores fila: si se implementa la
% ecuación en diferencias en C/C++, estos valores van directo al código.
[Gzn, Gzd] = tfdata(G_z_zoh, 'v');

% Mostramos las tres representaciones en consola
disp('--- G(s) continuo ---');      display(G_s);
disp('--- G(z) ZOH ---');          display(G_z_zoh);
disp('--- G(z) Impulse Inv. ---'); display(G_z_imp);

%% 4. MAPAS DE POLOS Y CEROS EN EL PLANO Z
% El círculo unitario es la frontera de estabilidad discreta:
%   |z| < 1  →  polo estable  (respuesta decae en el tiempo)
%   |z| > 1  →  polo inestable (respuesta crece indefinidamente)
%   |z| = 1  →  límite (oscilación sostenida)
figure;
subplot(2,1,1);
zplane(cell2mat(G_z_zoh.num), cell2mat(G_z_zoh.den));
grid on;
title('Polos y Ceros en Z  –  Método ZOH');

subplot(2,1,2);
zplane(cell2mat(G_z_imp.num), cell2mat(G_z_imp.den));
grid on;
title('Polos y Ceros en Z  –  Método Impulse Invariant');

%% 5. COMPARACIÓN: RESPUESTA AL ESCALÓN
% Con Ts = 1 ms (muy pequeño frente a la dinámica del sistema), los tres
% coinciden casi perfectamente. Aumentar Ts para ver las discrepancias.
figure;
step(G_s, 'b', G_z_zoh, 'r--', G_z_imp, 'g--');
grid on;
legend('Continuo G(s)', 'ZOH  G(z)', 'Impulse Inv.  G(z)');
title('Respuesta al Escalón – Continuo vs. Discreto');

%% 6. COMPARACIÓN: RESPUESTA AL IMPULSO
% ZOH NO preserva la respuesta al impulso (no es su objetivo).
% Impulse Invariant SÍ la preserva por construcción: notar la diferencia
% en la amplitud inicial entre ambos métodos discretos.
figure;
impulse(G_s, 'b', G_z_zoh, 'r--', G_z_imp, 'g--');
grid on;
legend('Continuo G(s)', 'ZOH  G(z)', 'Impulse Inv.  G(z)');
title('Respuesta al Impulso – Continuo vs. Discreto');

%% 7. APERTURA DEL MODELO SIMULINK COMPLEMENTARIO
% S2Z_1_Sim.slx lee del workspace las variables Ts, G_s, G_z_zoh y G_z_imp.
% Permite visualizar dentro de Simulink el efecto del bloque ZOH sobre la
% señal de control tal como ocurre en un sistema embebido real.
% IMPORTANTE: ejecutar las secciones anteriores ANTES de abrir el modelo.
open_system('S2Z_1_Sim');
