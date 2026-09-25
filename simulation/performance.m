%% =========================================================================
% Full Simulation: AVERAGED Model (Quasi-Static) v3
% =========================================================================
% This script uses an averaged model (neglecting high-frequency electrical
% switching) to maintain computational feasibility.
%
% FIXES:
%   (1) Complete track velocity profile definition
%   (2) Line-to-line resistance accounting (2 * R_s)
% =========================================================================

clear; clc; close all;

% Load motor parameters
motor_parameters_opt;

%% =========================================================================
% 1. INVERSE DYNAMICS SIMULATION (REFERENCE GENERATION)
% =========================================================================
fprintf('--- (Part 1) Running Inverse Mechanical Simulation ---\n');

% --- Vehicle and Track Parameters ---
g         = 9.81;            % Gravitational acceleration (m/s^2)
rho       = 1.225;           % Air density (kg/m^3)
M         = 85;              % Total mass (vehicle + driver) (kg)
r_w       = 0.254;           % Wheel radius (m)
S         = 0.43971171;      % Frontal cross-sectional area (m^2)
Cx        = 0.10473777;      % Aerodynamic drag coefficient
CdA       = S * Cx;          % Drag area Cd * A (m^2)
C_rr_base = 0.005;           % Baseline rolling resistance coefficient
J_motor   = 0.5 * m * r^2;   % Motor/wheel rotational inertia (kg*m^2)
B         = 0.05;            % Viscous friction coefficient (N*m*s/rad)

% Total equivalent inertia reflected to motor shaft
J_total   = J_motor + (M - m) * r_w^2;

% --- Distance-Based Velocity Profile Definition ---
dist_total   = 850;
d_step       = 0.1;
d_vec        = (0:d_step:dist_total)';
n_points_ref = length(d_vec);
v_target_ref = zeros(n_points_ref, 1);

v_recta      = 25 / 3.6;     % Straightaway target speed (25 km/h)
v_bajada     = 20 / 3.6;     % Downhill target speed (20 km/h)
v_subida     = 18 / 3.6;     % Uphill target speed (18 km/h)

% --- FIX 1: FULL VELOCITY PROFILE DEFINITION ---
% 1. Segment-wise track velocity profile
v_target_ref(d_vec < 200)                         = v_recta;
v_target_ref(d_vec >= 200 & d_vec < 300)          = v_bajada;
v_target_ref(d_vec >= 300 & d_vec < 630)          = v_recta;
v_target_ref(d_vec >= 630 & d_vec < 700)          = v_subida;
v_target_ref(d_vec >= 700)                        = v_recta;

% 2. Overwrite starting section with acceleration ramp
dist_arranque              = 30;
idx_arranque               = d_vec <= dist_arranque;
n_arranque                 = sum(idx_arranque);
v_target_ref(idx_arranque) = linspace(0.1, v_recta, n_arranque); % Starts at 0.1 m/s

% 3. Smooth and clamp
v_profile_ref = smooth(v_target_ref, 100, 'moving');
v_profile_ref(v_profile_ref < 0.1) = 0.1;

% --- Kinematics and Required Reference Torque Calculation ---
dv_ds_ref     = gradient(v_profile_ref, d_step);
a_profile_ref = dv_ds_ref .* v_profile_ref;

% --- FIX 2: ROBUST TIME VECTOR CALCULATION ---
dt_vec_ref = d_step ./ v_profile_ref;
t_vec_ref  = cumsum(dt_vec_ref);
t_vec_ref  = t_vec_ref - t_vec_ref(1); % Ensure t(1) = 0

T_motor_req_ref = zeros(n_points_ref, 1);

for i = 1:n_points_ref
    [alpha, Crr] = get_track_profile(d_vec(i), C_rr_base);
    
    F_rodadura       = Crr * M * g * cos(alpha);
    F_aero           = 0.5 * rho * CdA * v_profile_ref(i)^2;
    F_pendiente      = M * g * sin(alpha);
    F_inercia        = M * a_profile_ref(i);
    
    T_resist_total   = (F_rodadura + F_aero + F_pendiente) * r_w;
    T_inercial       = J_total * (a_profile_ref(i) / r_w);
    T_friccion_motor = B * (v_profile_ref(i) / r_w);
    
    T_motor_req_ref(i) = T_resist_total + T_inercial + T_friccion_motor;
end

fprintf('--- (Part 1) Reference profiles generated successfully. ---\n');
fprintf('Target lap time: %.2f s\n', t_vec_ref(end));

%% =========================================================================
% 2. MOTOR AND CONTROLLER SIMULATION PARAMETERS
% =========================================================================
% --- Motor Parameters ---
p.V_dc      = V_dc;
p.R_s       = R_s;                 % Single-phase resistance (Ohms)
p.R_total   = 2 * p.R_s;           % FIX: Line-to-line resistance across two active phases
p.P         = P;                   % Pole pairs
p.lambda_m  = lambda;              % Rotor flux linkage (Wb)
p.K_e       = p.P * p.lambda_m;    % Back-EMF constant (V*s/rad)
p.K_t       = p.K_e;               % Torque constant (N*m/A)
p.J_total   = J_total;
p.B         = B;

% --- Physical Parameters (for T_L) ---
p.m         = M;
p.r_w       = r_w;
p.CdA       = CdA;
p.C_rr_base = C_rr_base;
p.g         = g;
p.rho       = rho;

% --- PI Controller Parameters ---
p.Kp        = 0.5;
p.Ki        = 1.0;

% --- Package Reference Profiles ---
p.t_ref     = t_vec_ref;
p.v_ref     = v_profile_ref;
p.d_ref     = d_vec;
p.T_req_ref = T_motor_req_ref;

%% =========================================================================
% 3. MOTOR DYNAMICS EXECUTION (AVERAGED MODEL)
% =========================================================================
fprintf('--- (Part 2) Running Motor Simulation (Averaged Model) ---\n');

t_span  = [0, t_vec_ref(end)];
x0      = [0.1 / p.r_w; 0]; % Initial condition: starts at 0.1 m/s
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6, 'Events', @(t,x) simStopEvent(t,x,p));

[t, x]  = ode45(@(t,x) modelo_motor_promediado(t, x, p), t_span, x0, options);

fprintf('--- (Part 2) Motor simulation completed. ---\n');

%% =========================================================================
% 4. POST-PROCESSING AND ENERGY CALCULATIONS
% =========================================================================
omega_m_actual    = x(:, 1);
error_integral    = x(:, 2);
v_actual_ms       = omega_m_actual * p.r_w;
v_actual_kmh      = v_actual_ms * 3.6;

n_puntos_sim      = length(t);
Te_actual         = zeros(n_puntos_sim, 1);
I_bateria_consumo = zeros(n_puntos_sim, 1);
V_aplicado_eff    = zeros(n_puntos_sim, 1);
T_L_real          = zeros(n_puntos_sim, 1);
Duty              = zeros(n_puntos_sim, 1);

v_ref_interp      = interp1(p.t_ref, p.v_ref, t);
T_req_ref_interp  = interp1(p.t_ref, p.T_req_ref, t);
d_ref_interp      = interp1(p.t_ref, p.d_ref, t);

for i = 1:n_puntos_sim
    omega_m_t = omega_m_actual(i);
    
    % --- Recalculate Actual Load Torque (T_L) ---
    [alpha, Crr] = get_track_profile(d_ref_interp(i), p.C_rr_base);
    F_roll       = Crr * p.m * p.g * cos(alpha);
    F_aero       = 0.5 * p.rho * p.CdA * (omega_m_t * p.r_w)^2;
    F_grade      = p.m * p.g * sin(alpha);
    T_L_real(i)  = (F_roll + F_aero + F_grade) * p.r_w + p.B * omega_m_t;
    
    % --- Recalculate Controller Action ---
    error          = (v_ref_interp(i) / p.r_w) - omega_m_t;
    duty_cycle_raw = p.Kp * error + p.Ki * error_integral(i);
    duty_cycle     = max(0, min(1, duty_cycle_raw));
    Duty(i)        = duty_cycle;
    
    if T_req_ref_interp(i) <= 0
        duty_cycle = 0;
    end
    V_aplicado_eff(i) = p.V_dc * duty_cycle;
    
    % --- Recalculate Electrical State (Algebraic) ---
    e_bemf = p.K_e * omega_m_t;
    
    I_prom = 0;
    % FIX: Use p.R_total (line-to-line 2*R_s)
    if (V_aplicado_eff(i) > e_bemf)
        I_prom = (V_aplicado_eff(i) - e_bemf) / p.R_total;
    end
    I_prom       = max(0, I_prom); % Non-regenerative
    Te_actual(i) = p.K_t * I_prom;
    
    % Power balance: P_in = P_out (mechanical) + P_loss (copper)
    % FIX: Use p.R_total for resistive dissipation
    P_mecanica = Te_actual(i) * omega_m_t;
    P_perdidas = I_prom^2 * p.R_total;
    P_in       = P_mecanica + P_perdidas;
    
    I_bateria_consumo(i) = P_in / p.V_dc;
    if I_bateria_consumo(i) < 0
        I_bateria_consumo(i) = 0;
    end
end

% --- Energy and Efficiency Calculations ---
P_bateria_W       = p.V_dc * I_bateria_consumo;
Energia_J         = trapz(t, P_bateria_W);
Energia_Wh        = Energia_J / 3600;
Energia_kWh       = Energia_Wh / 1000;
Distancia_km      = dist_total / 1000;
Eficiencia_km_kWh = Distancia_km / Energia_kWh;

fprintf('\n--- FINAL LAP RESULTS ---\n');
fprintf('Total Energy Consumed: %.2f Wh\n', Energia_Wh);
fprintf('Distance Traveled:     %.2f km\n', Distancia_km);
fprintf('Energy Efficiency:     %.2f km/kWh\n', Eficiencia_km_kWh);
fprintf('----------------------------------------\n');

%% =========================================================================
% 5. RESULTS VISUALIZATION
% =========================================================================
figure('Name', 'Motor Simulation (Averaged Model)', ...
       'NumberTitle', 'off', 'WindowState', 'maximized');

% Velocity Profile
subplot(2, 2, 1);
plot(t, v_actual_kmh, 'b', 'LineWidth', 2);
hold on;
plot(p.t_ref, p.v_ref * 3.6, 'r--', 'LineWidth', 1.5);
title('Velocity Profile (Actual vs. Target)');
xlabel('Time (s)');
ylabel('Velocity (km/h)');
legend('Actual Velocity (Simulated)', 'Target Velocity (Reference)', 'Location', 'best');
grid on;

% Torque Profile
subplot(2, 2, 2);
plot(t, Te_actual, 'b', 'LineWidth', 2);
hold on;
plot(t, T_req_ref_interp, 'r--', 'LineWidth', 1.5);
plot(t, T_L_real, 'g:');
title('Torque Profile (Delivered vs. Required)');
xlabel('Time (s)');
ylabel('Torque (N\cdot m)');
legend('T_e Delivered (Actual)', 'T_e Required (Reference)', 'T_L Actual Load (Resistive)', 'Location', 'best');
grid on;

% Battery Current Draw
subplot(2, 2, 3);
plot(t, I_bateria_consumo, 'r', 'LineWidth', 1.5);
title('Battery Current Draw (No Regen)');
xlabel('Time (s)');
ylabel('Current (A)');
grid on;

% Effective Voltage
subplot(2, 2, 4);
plot(t, V_aplicado_eff, 'm', 'LineWidth', 1.5);
title('Effective Applied Voltage (V_{dc} \times Duty Cycle)');
xlabel('Time (s)');
ylabel('Voltage (V)');
ylim([0, p.V_dc + 2]);
grid on;

% Save simulation outputs
save('p_current.mat', 't', 'I_bateria_consumo', 'Duty');

%% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function [value, isterminal, direction] = simStopEvent(t, ~, p)
    % Halts simulation when current time reaches target reference time
    value      = t - p.t_ref(end);
    isterminal = 1;
    direction  = 0;
end

function dx = modelo_motor_promediado(t, x, p)
    % State vector: x = [omega_m; error_integral]
    omega_m        = x(1);
    error_integral = x(2);
    
    % --- 1. Interpolate References ---
    v_ref_t     = interp1(p.t_ref, p.v_ref, t);
    omega_ref_t = v_ref_t / p.r_w;
    T_req_ref_t = interp1(p.t_ref, p.T_req_ref, t);
    d_now       = interp1(p.t_ref, p.d_ref, t);
    
    % --- 2. Calculate Actual Resistive Load (T_L) ---
    [alpha, Crr] = get_track_profile(d_now, p.C_rr_base);
    F_roll       = Crr * p.m * p.g * cos(alpha);
    F_aero       = 0.5 * p.rho * p.CdA * (omega_m * p.r_w)^2;
    F_grade      = p.m * p.g * sin(alpha);
    T_L_actual   = (F_roll + F_aero + F_grade) * p.r_w + p.B * omega_m;
    
    % --- 3. Velocity PI Controller ---
    error               = omega_ref_t - omega_m;
    d_error_integral_dt = error;
    
    duty_cycle_raw = p.Kp * error + p.Ki * error_integral;
    duty_cycle     = max(0, min(1, duty_cycle_raw));
    
    if T_req_ref_t <= 0
        duty_cycle          = 0;
        d_error_integral_dt = 0;
    end
    
    V_aplicado = p.V_dc * duty_cycle;
    
    % --- 4. Algebraic Electrical Calculation ---
    e_bemf = p.K_e * omega_m;
    
    I_prom = 0;
    % FIX: Use p.R_total (line-to-line 2*R_s)
    if (V_aplicado > e_bemf)
        I_prom = (V_aplicado - e_bemf) / p.R_total;
    end
    
    I_prom = max(0, I_prom); % Non-regenerative
    Te     = p.K_t * I_prom;
    
    % --- 5. Mechanical Dynamics ---
    d_omega_m_dt = (1 / p.J_total) * (Te - T_L_actual);
    
    dx = [d_omega_m_dt; d_error_integral_dt];
end

function [alpha, Crr] = get_track_profile(distancia_m, Crr_base)
    % 850 m track circuit profile
    d = mod(distancia_m, 850);
    
    % --- Elevation Slope (alpha) in radians ---
    if d >= 200 && d < 250         % Turn 1-2 (200 m) - DOWNHILL
        alpha = deg2rad(-3);       % -3 degrees slope
    elseif d >= 600 && d < 650     % Turn 3-4 (200 m) - UPHILL
        alpha = deg2rad(10);       % +10 degrees slope
    else                           % Straightaways
        alpha = 0;
    end
    
    % --- Rolling Resistance Coefficient (Crr) - DIP / SPEED BUMP SIMULATION ---
    Crr = Crr_base;
    if d >= 100 && d < 105         % Dip in the first straightaway (5 m length)
        Crr = Crr_base * 5;        % Temporarily increases friction 5x
    end
end