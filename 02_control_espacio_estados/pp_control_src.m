% =========================================================================
% Control en Espacio de Estados (3x3, CONTINUO) - Posición angular
% del Motor BDC. A partir de este capítulo se usa el modelo 3x3 con
% theta como tercer estado y C = [0 0 1] (medición de posición).
% Parámetros didácticos: Ra=0.5, La=0.5, Kb=0.01, Je=0.01, Be=0.1
% =========================================================================

clear; clc; close all;

%% 1. ESPECIFICACIONES DE DISEÑO (Para el comportamiento dominante)
disp('--- 1. Especificaciones de Diseño ---');
Mp = 0.40;  % Sobreimpulso máximo (40%)
tp = 2;   % Tiempo pico deseado (s)

zeta = -log(Mp) / sqrt(pi^2 + (log(Mp))^2);
wn = pi / (tp * sqrt(1 - zeta^2));

fprintf('Factor de amortiguamiento (zeta) : %.4f\n', zeta);
fprintf('Frecuencia natural (wn)          : %.4f rad/s\n', wn);

%% 2. SELECCIÓN DE LOS 3 POLOS
disp(' ');
disp('--- 2. Polos Deseados Lazo Cerrado ---');

% Polos dominantes (los que dan el comportamiento deseado)
sigma = zeta * wn;
wd = wn * sqrt(1 - zeta^2);

p1 = -sigma + 1i*wd; 
p2 = -sigma - 1i*wd;

% EL TERCER POLO: Lo colocamos 10 veces más rápido (más negativo) que sigma
% para que no afecte el sobreimpulso ni el tiempo pico.
p3 = -10 * sigma; 

P = [p1; p2; p3];
disp('Vector de polos a ubicar:');
disp(P);

%% 3. MODELO 3x3 (MOTOR DC CON POSICIÓN)
% Parámetros físicos
Ra = 0.5;     % Resistencia (Ohms)
La = 0.5;     % Inductancia (H)
Kb = 0.01;    % Constante del motor
Je = 0.01;    % Inercia (kg*m^2)
Be = 0.1;     % Fricción (N*m*s)

% Vector de estados: x1 = Corriente (ia), x2 = Velocidad (we), x3 = Posición (theta)
% Ecuaciones:
% di_a/dt = (-Ra/La)*i_a + (-Kb/La)*we + (1/La)*V
% dwe/dt  = (Kb/Je)*i_a  + (-Be/Je)*we
% dtheta/dt = 1 * we

A = [-Ra/La, -Kb/La,  0 ; 
      Kb/Je, -Be/Je,  0 ;
          0,      1,  0 ]; % La derivada de la posición es la velocidad

B = [1/La ; 
        0 ; 
        0 ];

C = [0, 0, 1];   % ¡OJO! Ahora medimos la posición (theta), el 3er estado
D = 0;

sys_planta = ss(A, B, C, D);

%% 4. DISEÑO DEL CONTROLADOR
disp('--- 3. Matriz de Ganancias K (1x3) ---');
% Calculamos K para ubicar los 3 polos
K = place(A, B, P);
fprintf('K = [%.4f, %.4f, %.4f]\n', K(1), K(2), K(3));

%% 5. VALIDACIÓN EN LAZO CERRADO Y PRE-COMPENSACIÓN
A_cl = A - B*K;

% NOTA TÉCNICA: Aunque la planta original tiene un integrador natural
% (la relación velocidad-posición), al aplicar u = -Kx cerramos el lazo y
% movemos TODOS los polos, perdiendo el integrador en el origen. 
% Por lo tanto, seguimos necesitando K_dc para asegurar que theta = 1 cuando Ref = 1.

sys_cl_sin_ajuste = ss(A_cl, B, C, D);
K_dc = 1 / dcgain(sys_cl_sin_ajuste); 

% Sistema Final
sys_cl_final = ss(A_cl, B*K_dc, C, D);

% Función ideal (solo de referencia gráfica)
G_ideal = tf(wn^2, [1, 2*zeta*wn, wn^2]);

%% 6. GRÁFICOS
% Horizonte temporal adaptativo a las especificaciones (Mp, tp):
% - Cubre ~3 tiempos de asentamiento del 2 % ( Ts2% ≈ 4/(zeta*wn) ) y al
%   menos 2.5*tp para que el pico y el régimen permanente sean visibles.
% - El paso dt se elige para tener ~1000 muestras en la ventana.
Ts_set = 4 / (zeta*wn);                  % tiempo de asentamiento aprox.
Tend   = max(2.5*tp, 3*Ts_set);          % horizonte de simulación
dt     = Tend / 1000;                    % resolución temporal

figure('Name', sprintf('Control de Posición Motor DC (Mp=%.0f%%, tp=%.2fs)', Mp*100, tp), ...
       'Position', [100, 100, 900, 400]);

% Gráfico 1: Respuesta al Escalón
subplot(1,2,1);
step(G_ideal, 'k--', Tend); hold on;
step(sys_cl_final, 'b-', Tend);
title('Seguimiento de Posición Angular (\theta)');
legend('2do Orden Ideal', 'Motor 3x3 Controlado', 'Location', 'SouthEast');
xlabel('Tiempo (s)'); ylabel('Posición Angular (rad)');
xlim([0 Tend]);
grid on;

% Gráfico 2: Esfuerzo de Control (Voltaje aplicado)
% ¿Qué voltaje le estamos exigiendo al motor? Es vital para un ingeniero saberlo.
% u(t) = -K*x(t) + K_dc*r(t). Simularemos la entrada de referencia y calcularemos 'u'
subplot(1,2,2);
t = 0:dt:Tend;
r = ones(size(t)); % Escalón unitario de referencia
[y, t, x] = lsim(sys_cl_final, r, t); % Simulamos para obtener los estados 'x' en el tiempo
u = -x*K' + K_dc*r'; % Ley de control
plot(t, u, 'r', 'LineWidth', 1.5);
title('Esfuerzo de Control (Voltaje V_a)');
xlabel('Tiempo (s)'); ylabel('Voltaje (V)');
xlim([0 Tend]);
% Margen vertical del 10 % alrededor del rango observado de u(t)
u_min = min(u); u_max = max(u);
u_pad = 0.10 * max(abs(u_max - u_min), eps);
ylim([u_min - u_pad, u_max + u_pad]);
grid on;

%% 7. RESULTADOS NUMÉRICOS
info_real = stepinfo(sys_cl_final);
disp('--- 4. Verificación de Resultados ---');
fprintf('Sobreimpulso Esperado: %.2f %% | Obtenido: %.2f %%\n', Mp*100, info_real.Overshoot);
fprintf('Tiempo Pico Esperado : %.4f s  | Obtenido: %.4f s\n', tp, info_real.PeakTime);