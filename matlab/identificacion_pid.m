%% ════════════════════════════════════════════════════════════
%  IDENTIFICACIÓN AUTOMÁTICA Y DISEÑO DE PID
%  Puzzlebot - Mini Challenge 2
%% ════════════════════════════════════════════════════════════
clear; clc; close all;

%% ── PARÁMETROS DEL ROBOT (página 15 presentación) ───────────
r = 0.05;   % radio de ruedas (m)
l = 0.19;   % distancia entre ruedas (m)

%% ════════════════════════════════════════════════════════════
%  PARTE 1 — EXPERIMENTO LINEAL
%% ════════════════════════════════════════════════════════════

%% Cargar datos
data_v = readtable('experimento_lineal.csv');
t_v    = data_v.time;
wr_v   = data_v.VelocityEncR;
wl_v   = data_v.VelocityEncL;
cmd_v  = data_v.cmd_linear;

%% Calcular velocidad lineal real
% V = r * (ωR + ωL) / 2
v_real = r * (wr_v + wl_v) / 2;

%% Encontrar cuando empieza el escalón
idx_start_v = find(cmd_v > 0.01, 1, 'first');
t_start_v   = t_v(idx_start_v);

%% Calcular K y tau automáticamente
idx_cmd_v   = cmd_v > 0.01;
datos_cmd_v = v_real(idx_cmd_v);
v_final     = mean(datos_cmd_v(end - round(0.2*length(datos_cmd_v)):end));
v_cmd_valor = mean(cmd_v(idx_cmd_v));
K_v         = v_final / v_cmd_valor;
umbral_v    = 0.632 * v_final;
idx_tau_v   = find(v_real >= umbral_v, 1, 'first');
tau_v       = t_v(idx_tau_v) - t_start_v;

fprintf('=== EXPERIMENTO LINEAL ===\n');
fprintf('K_v   = %.4f\n', K_v);
fprintf('tau_v = %.4f s\n', tau_v);

%% Identificación con System Identification Toolbox
datos_v  = iddata(v_real, cmd_v, mean(diff(t_v)));
modelo_v = tfest(datos_v, 1);
K_v_id   = dcgain(modelo_v);
polo_v   = pole(modelo_v);
tau_v_id = -1 / polo_v;
G_v_id   = tf(modelo_v);

fprintf('K_v   identificado = %.4f\n', K_v_id);
fprintf('tau_v identificado = %.4f s\n', tau_v_id);

%% Diseño automático del PID lineal
opts_v        = pidtuneOptions('CrossoverFrequency', 1/tau_v_id, 'PhaseMargin', 60);
[C_v, info_v] = pidtune(G_v_id, 'pid', opts_v);
kp_v = C_v.Kp;
ki_v = C_v.Ki;
kd_v = C_v.Kd;

fprintf('\nGanancias PID Lineal:\n');
fprintf('kp_v = %.4f\n', kp_v);
fprintf('ki_v = %.4f\n', ki_v);
fprintf('kd_v = %.4f\n', kd_v);
fprintf('Margen de fase: %.1f degrees\n', info_v.PhaseMargin);
fprintf('Estable: %d\n', info_v.Stable);

%% Gráficas lineal
dt_v     = mean(diff(t_v));
t_unif_v = (0:length(t_v)-1)' * dt_v;

figure(1); clf;  % clf limpia figura antes de dibujar
subplot(2,1,1);
t_sim_v = linspace(0, max(t_v), 1000);
[y_sim_v, t_out_v] = step(feedback(C_v * G_v_id, 1) * v_cmd_valor, t_sim_v);
plot(t_v, v_real, 'b-', 'LineWidth', 2); hold on;
plot(t_out_v, y_sim_v, 'g--', 'LineWidth', 2);
xlim([0 max(t_v)]);
legend('Datos reales', 'Respuesta con PID');
xlabel('Tiempo (s)'); ylabel('V (m/s)');
title('Validacion PID Lineal'); grid on;

subplot(2,1,2);
[y_mod_v, t_mod_v] = lsim(G_v_id, cmd_v, t_unif_v);
plot(t_v, v_real, 'b-', 'LineWidth', 2); hold on;
plot(t_mod_v, y_mod_v, 'r--', 'LineWidth', 2);
xlim([0 max(t_v)]);
legend('Datos reales', 'Modelo identificado');
xlabel('Tiempo (s)'); ylabel('V (m/s)');
title('Validacion del Modelo Lineal'); grid on;


%% ════════════════════════════════════════════════════════════
%  PARTE 2 — EXPERIMENTO ANGULAR
%% ════════════════════════════════════════════════════════════

%% Cargar datos
data_w = readtable('experimento_angular.csv');
t_w    = data_w.time;
wr_w   = data_w.VelocityEncR;
wl_w   = data_w.VelocityEncL;
cmd_w  = data_w.cmd_angular;

%% Calcular velocidad angular real
% ω = r * (ωR - ωL) / l
w_real = r * (wr_w - wl_w) / l;

%% Encontrar cuando empieza el escalón
idx_start_w = find(abs(cmd_w) > 0.01, 1, 'first');
t_start_w   = t_w(idx_start_w);

%% Calcular K y tau automáticamente
idx_cmd_w   = abs(cmd_w) > 0.01;
datos_cmd_w = w_real(idx_cmd_w);
w_final     = mean(datos_cmd_w(end - round(0.2*length(datos_cmd_w)):end));
w_cmd_valor = mean(cmd_w(idx_cmd_w));
K_w         = w_final / w_cmd_valor;
umbral_w    = 0.632 * w_final;
idx_tau_w   = find(w_real >= umbral_w, 1, 'first');
tau_w       = t_w(idx_tau_w) - t_start_w;

fprintf('\n=== EXPERIMENTO ANGULAR ===\n');
fprintf('K_w   = %.4f\n', K_w);
fprintf('tau_w = %.4f s\n', tau_w);

%% Identificación con System Identification Toolbox
datos_w  = iddata(w_real, cmd_w, mean(diff(t_w)));
modelo_w = tfest(datos_w, 1);
K_w_id   = dcgain(modelo_w);
polo_w   = pole(modelo_w);
tau_w_id = -1 / polo_w;
G_w_id   = tf(modelo_w);

fprintf('K_w   identificado = %.4f\n', K_w_id);
fprintf('tau_w identificado = %.4f s\n', tau_w_id);

%% Diseño automático del PID angular
opts_w        = pidtuneOptions('CrossoverFrequency', 1/tau_w_id, 'PhaseMargin', 60);
[C_w, info_w] = pidtune(G_w_id, 'pid', opts_w);
kp_w = C_w.Kp;
ki_w = C_w.Ki;
kd_w = C_w.Kd;

fprintf('\nGanancias PID Angular:\n');
fprintf('kp_w = %.4f\n', kp_w);
fprintf('ki_w = %.4f\n', ki_w);
fprintf('kd_w = %.4f\n', kd_w);
fprintf('Margen de fase: %.1f degrees\n', info_w.PhaseMargin);
fprintf('Estable: %d\n', info_w.Stable);

%% Gráficas angular
dt_w     = mean(diff(t_w));
t_unif_w = (0:length(t_w)-1)' * dt_w;

figure(2); clf;  % clf limpia figura antes de dibujar
subplot(2,1,1);
t_sim_w = linspace(0, max(t_w), 1000);
[y_sim_w, t_out_w] = step(feedback(C_w * G_w_id, 1) * w_cmd_valor, t_sim_w);
plot(t_w, w_real, 'b-', 'LineWidth', 2); hold on;
plot(t_out_w, y_sim_w, 'g--', 'LineWidth', 2);
xlim([0 max(t_w)]);  % limita eje X al tiempo del experimento angular
legend('Datos reales', 'Respuesta con PID');
xlabel('Tiempo (s)'); ylabel('omega (rad/s)');
title('Validacion PID Angular'); grid on;

subplot(2,1,2);
[y_mod_w, t_mod_w] = lsim(G_w_id, cmd_w, t_unif_w);
plot(t_w, w_real, 'b-', 'LineWidth', 2); hold on;
plot(t_mod_w, y_mod_w, 'r--', 'LineWidth', 2);
xlim([0 max(t_w)]);  % limita eje X al tiempo del experimento angular
legend('Datos reales', 'Modelo identificado');
xlabel('Tiempo (s)'); ylabel('omega (rad/s)');
title('Validacion del Modelo Angular'); grid on;


%% ════════════════════════════════════════════════════════════
%  PARTE 3 — RESUMEN FINAL
%% ════════════════════════════════════════════════════════════
fprintf('\n========================================\n');
fprintf('GANANCIAS PID FINALES - COPIA EN ROS2\n');
fprintf('========================================\n');
fprintf("self.declare_parameter('kp_v', %.4f)\n", kp_v);
fprintf("self.declare_parameter('ki_v', %.4f)\n", ki_v);
fprintf("self.declare_parameter('kd_v', %.4f)\n", kd_v);
fprintf("self.declare_parameter('kp_w', %.4f)\n", kp_w);
fprintf("self.declare_parameter('ki_w', %.4f)\n", ki_w);
fprintf("self.declare_parameter('kd_w', %.4f)\n", kd_w);
