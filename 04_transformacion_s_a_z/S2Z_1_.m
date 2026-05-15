% Limpieza
clc;
clear;
close all;

% Units and constants
seg = 1;
mseg = seg * (10^-3);
Hz = 1/seg;
kHz = Hz * (10^3);

%# System parameters
Fs = 1 * kHz;  % Sampling frequency (1 kHz)
Ts = 1/Fs;     % Sampling period
N = 1024;

% Define the continuous-time transfer function
s = tf('s');
G_s = 1 / (2*s^2 + s + 5); % Example system

[Gn, Gd] = tfdata(G_s, 'v'); % 'v' returns them as row vectors

% Convert using Zero-Order Hold (ZOH)
G_z_zoh = c2d(G_s, Ts, 'zoh');
[Gzn, Gzd] = tfdata(G_z_zoh, 'v');

% Convert using Impulse Invariant method
G_z_imp = c2d(G_s, Ts, 'impulse');

% Display the transfer functions
display(G_s);
display(G_z_zoh);
display(G_z_imp);

% Plot pole-zero maps
figure;
subplot(2,1,1);
zplane(cell2mat(G_z_zoh.num), cell2mat(G_z_zoh.den));
grid on;
title('Pole-Zero Plot (ZOH Method)');

subplot(2,1,2);
zplane(cell2mat(G_z_imp.num), cell2mat(G_z_imp.den));
grid on;
title('Pole-Zero Plot (Impulse Invariant Method)');

% Step response comparison
figure;
step(G_s, 'b', G_z_zoh, 'r--', G_z_imp, 'g--');
grid on;
legend('Continuous', 'ZOH', 'Impulse Invariant');
title('Step Response Comparison');

% Impulse response comparison
figure;
impulse(G_s, 'b', G_z_zoh, 'r--', G_z_imp, 'g--');
grid on;
legend('Continuous', 'ZOH', 'Impulse Invariant');
title('Impulse Response Comparison');
