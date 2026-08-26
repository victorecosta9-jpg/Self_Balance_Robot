# Self_Balance_Robot
Trabalho de Conclusão de Curso Engenharia de Controle e Automação

// Ganhos LQR Discretos extraídos do MATLAB
float K_x     = -11.1056;
float K_theta = -87.4898;
float K_v     = -20.1484;
float K_w     = -15.1198;

// Cálculo do esforço de controle u (Tensão em Volts)
float u = -(K_x * position + K_theta * angle + K_v * velocity + K_w * angular_velocity);

// Ganhos LQR Discretos extraídos do MATLAB
const float K_x     = -11.1056f;
const float K_theta = -87.4898f;
const float K_v     = -20.1484f;
const float K_w     = -15.1198f;

// Tempo de amostragem configurado (500 Hz -> 2 ms)
const float Ts = 0.002f; 

// Cálculo da Lei de Controle
// Obs: Garanta que os estados sigam as unidades do SI (m, rad, m/s, rad/s)
float u = -(K_x * position + K_theta * angle + K_v * velocity + K_w * angular_velocity);
