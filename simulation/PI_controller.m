%% =========================================================================
% BLDC Hub Motor Simulation (Dynamic Model with PI Control)
% =========================================================================
% This script models the physical dynamics of a BLDC hub motor (including
% phase inductance L_s) governed by a closed-loop PI speed controller.
% =========================================================================

clear; clc; close all;

%% =========================================================================
% 1. MOTOR AND SIMULATION PARAMETERS
% =========================================================================
motor_parameters_350W;

% --- Electrical Parameters ---
% K_e = P * lambda;        % Back-EMF constant (V*s/rad)
K_t     = K_e;             % Torque constant (N*m/A)
R_total = 2 * R_s;         % Line-to-line resistance (active two-phase path)

% --- Mechanical Parameters ---
J   = J_r;                 % Total rotor/load inertia (kg*m^2)
B   = 0.08;                % Viscous friction coefficient (N*m*s/rad)
T_L = 0;                   % External load torque (N*m)

% --- Controller Parameters ---
Kp            = 5.0;       % Proportional gain
Ki            = 1.0;       % Integral gain
v_final_ms    = 8.3;       % Target linear speed (m/s)
t_aceleracion = 1.3;       % Acceleration ramp duration (s)
omega_final   = v_final_ms / r_w; % Target angular velocity (rad/s)

% --- Simulation Configuration ---
t_sim = 2;                 % Total simulation time (s)

% State vector layout: x = [omega_m; error_integral; i]
% x0 = [initial_omega_m; initial_error_integral; initial_current]
x0 = [0; 0; 0];

%% =========================================================================
% 2. SIMULATION EXECUTION
% =========================================================================
fprintf('Running dynamic model simulation (stiff system)...\n');

% Using ode15s for stiff systems combining fast electrical and slow mechanical dynamics
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t, x]  = ode15s(@(t,x) modelo_motor_bldc(t, x, V_dc, R_total, L_s, K_e, K_t, P, J, B, T_L, Kp, Ki, omega_final, t_aceleracion), ...
                 [0, t_sim], x0, options);

fprintf('Simulation completed successfully.\n');

%% =========================================================================
% 3. POST-PROCESSING AND ENERGY CALCULATIONS
% =========================================================================
% Extract state trajectories
omega_m        = x(:, 1);
error_integral = x(:, 2);
I              = x(:, 3);
rpm            = omega_m * (60 / (2 * pi));

% Reconstruct internal control and electrical signals
n_puntos       = length(t);
v_aplicado_vec = zeros(n_puntos, 1);
bemf_vec       = zeros(n_puntos, 1);
Te_vec         = zeros(n_puntos, 1);

for i = 1:n_puntos
    omega_m_i        = omega_m(i);
    error_integral_i = error_integral(i);
    
    % Reconstruct dynamic ramp reference
    t_actual = t(i);
    if t_actual < t_aceleracion
        omega_ref_actual = (omega_final / t_aceleracion) * t_actual;
    else
        omega_ref_actual = omega_final;
    end
    
    % Reconstruct PI controller action
    error          = omega_ref_actual - omega_m_i;
    duty_cycle_raw = Kp * error + Ki * error_integral_i;
    duty_cycle     = max(0, min(1, duty_cycle_raw));
    
    v_aplicado_vec(i) = V_dc * duty_cycle;
    bemf_vec(i)       = K_e * omega_m_i;
    Te_vec(i)         = K_t * I(i);
end

% --- Energy Consumption Calculations ---
Potencia_consumida = v_aplicado_vec .* I; 

% Exclude negative power (non-regenerative unidirectional drive)
Potencia_consumida(Potencia_consumida < 0) = 0; 
Energia_J  = trapz(t, Potencia_consumida);
Energia_Wh = Energia_J / 3600;

fprintf('\n--- DYNAMIC MOTOR SIMULATION RESULTS ---\n');
fprintf('Simulation Time:        %.2f s\n', t_sim);
fprintf('Total Energy Consumed:  %.2f J\n', Energia_J);
fprintf('Total Energy Consumed:  %.4f Wh\n', Energia_Wh);
fprintf('----------------------------------------\n');

%% =========================================================================
% 4. RESULTS VISUALIZATION
% =========================================================================
figure('Name', 'Dynamic BLDC Motor Simulation Results', ...
       'NumberTitle', 'off', 'WindowState', 'maximized');

% Motor Phase Current
subplot(2, 2, 1);
plot(t, I, 'r', 'LineWidth', 1.5);
title('Motor Phase Current (I)');
xlabel('Time (s)');
ylabel('Current (A)');
legend('Phase Current (i)', 'Location', 'best');
grid on;

% Motor Rotational Speed
subplot(2, 2, 2);
plot(t, rpm, 'b', 'LineWidth', 1.5);
hold on;
omega_ref_vec = zeros(size(t));
for i = 1:length(t)
    if t(i) < t_aceleracion
        omega_ref_vec(i) = (omega_final / t_aceleracion) * t(i);
    else
        omega_ref_vec(i) = omega_final;
    end
end
plot(t, omega_ref_vec * (60 / (2 * pi)), 'r--', 'LineWidth', 1.5);
title('Motor Speed Tracking');
xlabel('Time (s)');
ylabel('Speed (RPM)');
legend('Actual Speed', 'Target Speed', 'Location', 'best');
grid on;

% Electromagnetic Torque
subplot(2, 2, 3);
plot(t, Te_vec, 'k', 'LineWidth', 1.5);
hold on;
plot(t, T_L * ones(size(t)), 'm--', 'LineWidth', 1.5);
title('Electromagnetic Torque vs. Load');
xlabel('Time (s)');
ylabel('Torque (N\cdot m)');
legend('T_e (Delivered)', 'T_L (External Load)', 'Location', 'best');
grid on;

% Applied Voltage vs. Back-EMF
subplot(2, 2, 4);
plot(t, v_aplicado_vec, 'r', 'LineWidth', 1.5, 'DisplayName', 'V_{applied}');
hold on;
plot(t, bemf_vec, 'k--', 'LineWidth', 1.5, 'DisplayName', 'e_{BEMF}');
title('Effective Applied Voltage vs. Back-EMF');
xlabel('Time (s)');
ylabel('Voltage (V)');
legend('show', 'Location', 'best');
grid on;

%% =========================================================================
% HELPER FUNCTIONS: DYNAMIC BLDC MODEL
% =========================================================================

function dx = modelo_motor_bldc(t, x, V_dc, R_total, L_s, Ke, Kt, P, J, B, TL, Kp, Ki, omega_final, t_aceleracion)
    % State vector: x = [omega_m; error_integral; i]
    omega_m        = x(1);
    error_integral = x(2);
    i              = x(3);
    
    % --- Dynamic Reference Generation ---
    % Linear acceleration ramp
    if t < t_aceleracion
        omega_ref_actual = (omega_final / t_aceleracion) * t;
    else
        omega_ref_actual = omega_final;
    end
    
    % Parabolic acceleration alternative (optional):
    % if t < t_aceleracion
    %     k = omega_final / (t_aceleracion^2);
    %     omega_ref_actual = k * t^2;
    % else
    %     omega_ref_actual = omega_final;
    % end
    
    % --- Velocity PI Controller ---
    error               = omega_ref_actual - omega_m;
    d_error_integral_dt = error;
    
    duty_cycle_raw      = Kp * error + Ki * error_integral;
    duty_cycle          = max(0, min(1, duty_cycle_raw));
    
    V_aplicado          = V_dc * duty_cycle;
    
    % --- Back-Electromotive Force (Back-EMF) ---
    e_bemf              = Ke * omega_m;
    
    % --- Electrical Dynamics ---
    % di/dt = (1 / L) * (V - R * i - e_bemf)
    d_i_dt              = (1 / L_s) * (V_aplicado - R_total * i - e_bemf);
    
    % --- Non-Regenerative Constraint ---
    % If applied voltage is lower than Back-EMF, conduction depends on freewheeling diode state
    if (V_aplicado <= e_bemf)
        d_i_dt = 0; % Controller output clamped
        if i <= 0
            i      = 0;
            d_i_dt = 0; % Prevent negative reverse current
        else
            % Natural current decay across RL loop
            d_i_dt = (1 / L_s) * (0 - R_total * i - e_bemf);
        end
    end
    
    % --- Mechanical Dynamics ---
    Te            = Kt * i;
    T_carga_total = TL + B * omega_m;
    d_omega_m_dt  = (1 / J) * (Te - T_carga_total);
    
    % State derivatives matching initial conditions x0
    dx = [d_omega_m_dt; d_error_integral_dt; d_i_dt];
end