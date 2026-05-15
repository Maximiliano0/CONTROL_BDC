%% S <-> Z Domains & Frequency Response
% Professor: Eng. Maximiliano Vega
% Course: Discrete Control (UCA)
% Description: Visualizing S-Z mapping, G(s) vs G(z), Ts variation, ROC, and Periodicity.

clear; clc; close all;

% General System Definition for our time-domain examples
wn = 5;         % Natural frequency [rad/s]
zeta = 0.3;     % Damping ratio (Underdamped to see oscillations)
G_s = tf(wn^2, [1, 2*zeta*wn, wn^2]); % G(s) = 25 / (s^2 + 3s + 25)

% Base Sampling Time
Ts_base = 0.1; 

%% 1. S <-> Z PLANE CORRESPONDENCE (Mapping s = sigma + j*omega to z = e^{s*Ts})
figure('Name', '1. S-Plane to Z-Plane Correspondence', 'NumberTitle', 'off', 'Position', [50, 100, 1000, 400]);

% S-Plane Plot
subplot(1,2,1); hold on; grid on;
title('S-Plane: Constant \sigma and \omega lines');
xlabel('Real (\sigma)'); ylabel('Imaginary (j\omega)');
xlim([-10, 2]); ylim([-15, 15]);
xline(0, 'k', 'LineWidth', 2); % Imaginary axis

% Z-Plane Plot
subplot(1,2,2); hold on; grid on; axis equal;
title('Z-Plane: Mapped from S-Plane (z = e^{s T_s})');
xlabel('Real'); ylabel('Imaginary');
xlim([-1.5, 1.5]); ylim([-1.5, 1.5]);
% Draw Unit Circle
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k', 'LineWidth', 2); 

% Map Constant Sigma (Damping/Decay rate) -> Concentric circles in Z
sigmas = [-1, -3, -5, -8];
omega_range = linspace(-15, 15, 200);
colors = lines(length(sigmas));
for i = 1:length(sigmas)
    s_line = sigmas(i) + 1j*omega_range;
    z_line = exp(s_line * Ts_base);
    subplot(1,2,1); plot(real(s_line), imag(s_line), 'Color', colors(i,:), 'LineWidth', 1.5);
    subplot(1,2,2); plot(real(z_line), imag(z_line), 'Color', colors(i,:), 'LineWidth', 1.5);
end

% Map Constant Omega (Frequency) -> Radial lines in Z
omegas = [3, 6, 9, 12];
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
legend('Unit Circle (s=j\omega)', 'Mapped \sigma', 'Location', 'best');

%% 2. G(s) CONTINUOUS vs G(z) DISCRETE DOMAIN VIEW (ZOH)
figure('Name', '2. G(s) vs G*(s) vs G(z)', 'NumberTitle', 'off', 'Position', [100, 150, 800, 450]);

% Discretize G(s) using Zero-Order Hold (Microcontroller DAC behavior)
G_z = c2d(G_s, Ts_base, 'zoh');

t_cont = 0:0.005:3;
t_disc = 0:Ts_base:3;

[y_c, ~] = step(G_s, t_cont);
[y_d, ~] = step(G_z, t_disc);

hold on; grid on;
plot(t_cont, y_c, 'b', 'LineWidth', 2, 'DisplayName', 'Continuous G(s)');
stairs(t_disc, y_d, 'r', 'LineWidth', 1.5, 'DisplayName', 'Discrete G(z) (ZOH staircase)');
stem(t_disc, y_d, 'k', 'Filled', 'DisplayName', 'Ideal Samples G*(s)');
title(['Step Response Comparison: Continuous vs. Discrete (Ts = ', num2str(Ts_base), 's)']);
xlabel('Time [s]'); ylabel('Amplitude');
legend('Location', 'Southeast');

%% 3. POLE LOCATIONS OF G(z) WITH SAMPLING FREQUENCY (Ts) CHANGES
figure('Name', '3. Pole Migration with Ts Variation', 'NumberTitle', 'off', 'Position', [150, 200, 600, 600]);
hold on; grid on; axis equal;
zgrid; % Draw standard Z-plane grid (Damping and Frequency lines)
title('Migration of Z-plane Poles as Sampling Time (Ts) Increases');

% Sweep Ts from very fast (0.01s) to very slow (0.4s)
Ts_array = [0.01, 0.05, 0.1, 0.2, 0.3, 0.4];
colors = parula(length(Ts_array));

for i = 1:length(Ts_array)
    Ts_current = Ts_array(i);
    G_z_current = c2d(G_s, Ts_current, 'zoh');
    p_z = pole(G_z_current);
    
    % Plot poles
    plot(real(p_z), imag(p_z), 'x', 'MarkerSize', 10, 'LineWidth', 2, ...
         'Color', colors(i,:), 'DisplayName', ['Ts = ', num2str(Ts_current), ' s']);
end
legend('Location', 'southwest');

%% 4. CONVERGENCE AREA AND STABILITY BOUNDARIES (ROC)
figure('Name', '4. Convergence Area & Stability', 'NumberTitle', 'off', 'Position', [200, 250, 600, 600]);
hold on; grid on; axis equal;
title('Region of Convergence (ROC) and Stability in Z-Domain');
xlabel('Real'); ylabel('Imaginary');
xlim([-2, 2]); ylim([-2, 2]);

% Fill stable region (Inside unit circle)
fill(cos(theta), sin(theta), [0.8 1 0.8], 'EdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Stable Region (|z| < 1)');

% Fill Unstable region (Outside unit circle - bounded for visual purposes)
theta_outer = [theta, fliplr(theta)];
r_outer = [ones(1,100)*3, ones(1,100)];
x_outer = r_outer .* cos(theta_outer);
y_outer = r_outer .* sin(theta_outer);
fill(x_outer, y_outer, [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Unstable Region (|z| > 1)');

% Plot examples of stable and unstable poles
plot(0.8, 0.4, 'bx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Stable Pole (Decaying osc.)');
plot(0.95, 0, 'gx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Stable Pole (Slow decay)');
plot(1.1, 0.5, 'rx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Unstable Pole (Growing osc.)');
plot(-1.2, 0, 'kx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Unstable Pole (Growing ringing)');

legend('Location', 'northeast');

%% 5. BODE PLOT SHOWING fs/2 PERIODICITY (ALIASING SPECTRA)
figure('Name', '5. Frequency Response & Periodicity', 'NumberTitle', 'off', 'Position', [250, 300, 900, 600]);

% Parameters for Frequency Response
fs = 10; % Sampling frequency [Hz]
Ts_bode = 1/fs; 
fc = 2;  % Filter cutoff frequency [Hz]
wc = 2*pi*fc;

% Continuous Filters
LPF_s = tf(wc, [1, wc]);       % Low-Pass: 1 / (s + wc)
HPF_s = tf([1, 0], [1, wc]);   % High-Pass: s / (s + wc)

% Discretize using Tustin (Bilinear) to prevent frequency warping at high frequencies
LPF_z = c2d(LPF_s, Ts_bode, 'tustin');
HPF_z = c2d(HPF_s, Ts_bode, 'tustin');

% Frequency vector extending up to 3 times the sampling frequency
f_vec = linspace(0, 3*fs, 1000); % 0 to 30 Hz
w_vec = 2*pi*f_vec;              % 0 to 30*2*pi rad/s

% Evaluate Frequency Response Manually (MATLAB's bode cuts off at Nyquist)
% H(e^{j*w*Ts})
H_LPF = squeeze(freqresp(LPF_z, w_vec));
H_HPF = squeeze(freqresp(HPF_z, w_vec));

Mag_LPF_dB = 20*log10(abs(H_LPF));
Mag_HPF_dB = 20*log10(abs(H_HPF));

% Plot Magnitude Responses
subplot(2,1,1); hold on; grid on;
plot(f_vec, Mag_LPF_dB, 'b', 'LineWidth', 2);
title('Discrete Low-Pass Filter Magnitude Response (Notice Periodicity!)');
ylabel('Magnitude [dB]');
ylim([-40 5]);
% Draw vertical lines for Nyquist and Sampling Frequencies
for k = 1:3
    xline(k*fs/2, 'k--', ['k \cdot f_s/2 = ', num2str(k*fs/2), ' Hz'], 'LabelOrientation', 'horizontal');
end

subplot(2,1,2); hold on; grid on;
plot(f_vec, Mag_HPF_dB, 'r', 'LineWidth', 2);
title('Discrete High-Pass Filter Magnitude Response');
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
ylim([-40 5]);
for k = 1:3
    xline(k*fs/2, 'k--', ['k \cdot f_s/2 = ', num2str(k*fs/2), ' Hz'], 'LabelOrientation', 'horizontal');
end

disp('MATLAB Execution Complete. Please review the 5 generated figures.');