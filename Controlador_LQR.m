% PARÂMETROS %
M = 1.187;         % Kg - Massa do Corpo
m = 0.03825;       % Kg - Massa de uma roda (massa total das 2 rodas / 2)
Jc = 0.001;        % Kgm² - Momento de inércia do corpo (IoxG)
r = 0.03415;       % m - Raio da roda (Diâmetro 68,3 mm / 2)
Jr = 0.00002224;   % Kgm² - Momento de inércia das rodas
l = 0.037792;      % m - Distância entre o centro de massa e o eixo
Kr = 0.216;        % Vs/rad - Constante de rotação do motor
Kt = 0.312;        % Nm/A - Constante de torque do motor 
R_motor = 12;      % Ohm - Resistência elétrica do motor
g = 9.8;           % m/s² - Aceleração da gravidade
b = 0.000673;      % Nms/rad - Constante de atrito viscoso

% MODELO %
den = M^2 * l^2 * r^2 - (Jc + M * l^2) * (2 * Jr + r^2 * (M + 2 * m));

alpha = (M^2 * g * l^2 * r^2) / den;
beta = (2 * (Kr * Kt * M * l * r + (Jc + M * l^2) * (Kr * Kt + R_motor * b))) / (R_motor * den);
gamma = -(M * g * l * (2 * Jr + r^2 * (M + 2 * m))) / den;
delta = -(2 * Kr * Kt * (2 * Jr + r^2 * (M + 2 * m)) + 2 * M * l * r * (Kr * Kt + R_motor * b)) / (R_motor * r * den);
epsilon = -2 * Kt * r * (Jc + M * l^2 + M * l * r) / (R_motor * den);
zeta = (2 * Kt * (2 * Jr + M * l * r + r^2 * (M + 2 * m))) / (R_motor * den);

% ESPAÇO DE ESTADOS %
A = [0 0 1 0;
    0 0 0 1;
    0 alpha 0 beta;
    0 gamma 0 delta];

B = [0;
    0;
    epsilon;
    zeta];

C = [1 0 0 0;
    0 1 0 0];

D = [0; 0];

% SISTEMA CONTÍNUO E DISCRETO %
G = ss(A, B, C, D);
Ts = 0.01; % Tempo de amostragem [s]
Gz = c2d(G, Ts);

Ad = Gz.A;
Bd = Gz.B;

% ANÁLISE DE ESTABILIDADE %
poles_ma = eig(A);
poles_d_ma = eig(Ad);

% CONTROLABILIDADE %
Co = ctrb(A, B);
Co_d = ctrb(Ad, Bd);

% CONTROLE LQR COM CORREÇÃO DE SATURAÇÃO %
x_max = 0.5;              % [m]
theta_max = deg2rad(5);   % [rad]
v_max = 1.0;              % [m/s]
w_max = 2.0;              % [rad/s]
u_max = 12;               % [V]

Q = diag([1/x_max^2, 1/theta_max^2, 1/v_max^2, 1/w_max^2]);

% Ajuste no R: Aumentando o peso para limitar o pico de tensão em <= 12V
fator_r = 2.0;
R_lqr = fator_r * (1/u_max^2);

% GANHOS LQR %
Klqr = lqr(A, B, Q, R_lqr);
Klqr_z = lqrd(A, B, Q, R_lqr, Ts);

disp('================ GANHO DO CONTROLADOR LQR DISCRETO ================');
disp('Klqr_z =');
disp(Klqr_z);

% MALHA FECHADA %
Acl = A - B * Klqr;
Ad_cl = Ad - Bd * Klqr_z;

poles_mf = eig(Acl);
poles_d_mf = eig(Ad_cl);

% SIMULAÇÃO TEMPORAL %
t = 0:Ts:3;
x0 = [0; deg2rad(5); 0; 0];

sys_cl = ss(Acl, B, C, D);
[y, t, x] = initial(sys_cl, x0, t);
u = -Klqr * x';

% =========================================================================
% PLOT 1: RESPOSTA TEMPORAL
% =========================================================================
figure('Name', 'Simulação LQR - Resposta Temporal');
subplot(3,1,1);
plot(t, rad2deg(x(:,2)), 'b', 'LineWidth', 1.5);
grid on; ylabel('\theta [graus]'); title('Ângulo de Inclinação');

subplot(3,1,2);
plot(t, x(:,1), 'g', 'LineWidth', 1.5);
grid on; ylabel('x [m]'); title('Posição Linear do Robô');

subplot(3,1,3);
plot(t, u, 'r', 'LineWidth', 1.5);
hold on;
yline(12, 'k--', 'Limite +12V');
yline(-12, 'k--', 'Limite -12V');
grid on; ylabel('Tensão [V]'); xlabel('Tempo [s]');
title('Esforço de Controle (Tensão nos Motores)');

% =========================================================================
% PLOT 2: MAPA DE POLOS (PLANO S E PLANO Z)
% =========================================================================
figure('Name', 'Análise de Polos - Malha Aberta vs Malha Fechada');

% --- Plano S (Contínuo) ---
subplot(1,2,1);
plot(real(poles_ma), imag(poles_ma), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Malha Aberta');
hold on;
plot(real(poles_mf), imag(poles_mf), 'bo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Malha Fechada (LQR)');
xline(0, 'k--'); % Eixo de estabilidade
grid on; axis equal;
xlabel('Eixo Real (\sigma)'); ylabel('Eixo Imaginário (j\omega)');
title('Polos no Plano S (Contínuo)');
legend('Location', 'northwest');

% --- Plano Z (Discreto) ---
subplot(1,2,2);
% Desenhar círculo unitário
th = 0:pi/50:2*pi;
plot(cos(th), sin(th), 'k--', 'DisplayName', 'Círculo Unitário');
hold on;
plot(real(poles_d_ma), imag(poles_d_ma), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Malha Aberta');
plot(real(poles_d_mf), imag(poles_d_mf), 'bo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Malha Fechada (LQR)');
grid on; axis equal;
xlabel('Eixo Real'); ylabel('Eixo Imaginário');
title('Polos no Plano Z (Discreto)');
legend('Location', 'northwest');
