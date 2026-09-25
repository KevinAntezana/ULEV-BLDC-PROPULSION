%% =========================================================================
% Inverse Dynamics Simulation (Vehicle Loads)
% =========================================================================
% This script computes vehicle forces and required load torque (T_L)
% based on a defined TARGET VELOCITY PROFILE along the track.
% =========================================================================

clear; clc; close all;

% Load motor parameters
motor_parameters_350W;

%% =========================================================================
% 1. VEHICLE AND TRACK PARAMETERS (PROTOTYPE ESTIMATES)
% =========================================================================
g         = 9.81;            % Gravitational acceleration (m/s^2)
rho       = 1.225;           % Air density (kg/m^3)
M         = 85;              % Total mass (vehicle + driver) (kg)
r_w       = 0.254;           % Wheel radius (m)
S         = 0.43971171;      % Frontal cross-sectional area (m^2)
Cx        = 0.10473777;      % Aerodynamic drag coefficient
CdA       = S * Cx;          % Drag area Cd * A (m^2)
C_rr_base = 0.005;           % Baseline rolling resistance coefficient
J_motor   = 0.5 * m * r^2;   % Motor/wheel rotational inertia (kg*m^2)
B         = 0.05;            % Motor viscous friction (N*m*s/rad)

% Total equivalent inertia reflected to motor shaft
J_total   = J_motor + (M - m) * r_w^2;

%% =========================================================================
% 2. VELOCITY PROFILE DEFINITION (vs. DISTANCE)
% =========================================================================
% Define the target velocity along each track segment
dist_total   = 850;          % Total track distance (m)
d_step       = 0.1;          % Spatial resolution (10 cm)
d_vec        = (0:d_step:dist_total)';
n_points     = length(d_vec);
v_target     = zeros(n_points, 1);

% Target speeds (m/s)
v_recta      = 25 / 3.6;     % Straightaways (25 km/h)
v_bajada     = 20 / 3.6;     % Turn 1-2 Downhill (20 km/h)
v_subida     = 18 / 3.6;     % Turn 3-4 Uphill (18 km/h)

% Assign target speeds by track section
v_target(d_vec < 200)                         = v_recta;   % Straight 1
v_target(d_vec >= 200 & d_vec < 250)          = v_bajada;  % Turn 1-2 (Downhill)
v_target(d_vec >= 250 & d_vec < 600)          = v_recta;   % Straight 2
v_target(d_vec >= 600 & d_vec < 650)          = v_subida;  % Turn 3-4 (Uphill)
v_target(d_vec >= 650)                        = v_recta;   % End of Straight 1

% Startup simulation (Linear acceleration over the first 30 m)
dist_arranque              = 30;
idx_arranque               = d_vec <= dist_arranque;
n_arranque                 = sum(idx_arranque);
v_target(idx_arranque)     = linspace(0, v_recta, n_arranque);

% Smooth profile to achieve realistic transition accelerations
% span = 100 points -> 10-meter smoothing moving window
v_profile    = smooth(v_target, 100, 'moving');
v_profile(1) = 0; % Ensure starting speed is zero

%% =========================================================================
% 3. KINEMATICS CALCULATION (ACCELERATION AND TIME)
% =========================================================================
v_profile_m_s = v_profile;
v_profile_m_s(v_profile_m_s < 0.1) = 0.1; % Prevent division by zero

% Calculate acceleration: a(s) = v * (dv/ds)
dv_ds          = gradient(v_profile_m_s, d_step);
a_profile_m_s2 = dv_ds .* v_profile_m_s;

% Calculate time vector: t(s) = integral(ds / v(s))
dt_vec    = d_step ./ v_profile_m_s;
dt_vec(1) = 0; % Initial time is 0
t_vec_s   = cumsum(dt_vec);

fprintf('Simulated lap time: %.2f seconds\n', t_vec_s(end));
fprintf('Average speed:      %.2f km/h\n', (dist_total / t_vec_s(end)) * 3.6);

%% =========================================================================
% 4. FORCES AND LOAD TORQUE CALCULATION (vs. TIME)
% =========================================================================
% Pre-allocate result vectors
F_rodadura        = zeros(n_points, 1);
F_aero            = zeros(n_points, 1);
F_pendiente       = zeros(n_points, 1);
F_inercia         = zeros(n_points, 1);
T_motor_requerido = zeros(n_points, 1);
alpha_vec         = zeros(n_points, 1);

for i = 1:n_points
    v_now = v_profile_m_s(i);
    a_now = a_profile_m_s2(i);
    d_now = d_vec(i);
    
    % Retrieve track geometry conditions
    [alpha, Crr] = get_track_profile(d_now, C_rr_base);
    alpha_vec(i) = alpha; % Store for plotting
    
    % 1. Rolling resistance force
    F_rodadura(i) = Crr * M * g * cos(alpha);
    
    % 2. Aerodynamic drag force
    F_aero(i) = 0.5 * rho * CdA * v_now^2;
    
    % 3. Grade resistance force
    F_pendiente(i) = M * g * sin(alpha);
    
    % 4. Inertial force (F = m*a)
    F_inercia(i) = M * a_now;
    
    % --- Total Required Motor Torque ---
    % Sum of all resistive torques + inertial torque
    
    % Resistive torques (Forces * wheel radius)
    T_resist_total = (F_rodadura(i) + F_aero(i) + F_pendiente(i)) * r_w;
    
    % Inertial torque (Total equivalent inertia * angular acceleration)
    alpha_m    = a_now / r_w;
    T_inercial = J_total * alpha_m;
    
    % Motor viscous friction torque (B * omega)
    omega_m          = v_now / r_w;
    T_friccion_motor = B * omega_m;
    
    % Total torque the motor must deliver (T_e)
    T_motor_requerido(i) = T_resist_total + T_inercial + T_friccion_motor;
end

%% =========================================================================
% 5. TRACK AND VEHICLE PARAMETERS VISUALIZATION (FIGURE 2)
% =========================================================================
% This figure displays a schematic view of the oval track colored by
% elevation slope, alongside a summary of vehicle parameters.
fig2 = figure('Name', 'Track Profile (Oval) and HUALKANA II Parameters', ...
              'NumberTitle', 'off', 'WindowState', 'maximized');
fig2.Color = 'w'; % White background

% --- Subplot 1: Schematic Oval Track Layout ---
ax_oval = subplot(2, 1, 1);
hold(ax_oval, 'on');
box(ax_oval, 'on');

% 1. Compute schematic 850m oval geometry
% Perimeter = 2 * L_s + 2 * C_s = 850 m
% Curve length = 50 m -> Straight length = 375 m
L_s = 375;
C_s = 50;
r_s = C_s / pi; % Schematic curve radius

x_coords = zeros(n_points, 1);
y_coords = zeros(n_points, 1);
z_coords = zeros(n_points, 1); % Used for 'surface' call

% Define boundary segments of the schematic oval
d1_lim = L_s;
d2_lim = L_s + C_s;
d3_lim = (2 * L_s) + C_s;

for i = 1:n_points
    d_now = d_vec(i);
    
    if d_now < d1_lim
        % Straightaway 1 (0 -> 375 m)
        x_coords(i) = d_now;
        y_coords(i) = 0;
        
    elseif d_now < d2_lim
        % Turn 1 (375 -> 425 m)
        d_c         = d_now - d1_lim;
        theta       = (d_c / C_s) * pi;
        x_coords(i) = L_s + r_s * sin(theta);
        y_coords(i) = r_s - r_s * cos(theta);
        
    elseif d_now < d3_lim
        % Straightaway 2 (425 -> 800 m)
        d_s         = d_now - d2_lim;
        x_coords(i) = L_s - d_s;
        y_coords(i) = 2 * r_s;
        
    else
        % Turn 2 (800 -> 850 m)
        d_c         = d_now - d3_lim;
        theta       = (d_c / C_s) * pi;
        x_coords(i) = 0 - r_s * sin(theta);
        y_coords(i) = r_s + r_s * cos(theta);
    end
end

% 2. Draw oval colored by slope angle
color_data = rad2deg(alpha_vec);

h_surf = surface([x_coords, x_coords], ...
                 [y_coords, y_coords], ...
                 [z_coords, z_coords], ...
                 'facecol', 'no', ...
                 'edgecol', 'interp', ...
                 'linewidth', 4);

% 3. Add markers for downhill/uphill sections
idx_bajada = (d_vec >= 375 & d_vec < 425);
idx_subida = (d_vec >= 800 & d_vec < 850);

plot(x_coords(idx_bajada), y_coords(idx_bajada), 'ro', 'MarkerSize', 6, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Downhill Zone (-3°)');
plot(x_coords(idx_subida), y_coords(idx_subida), 'bo', 'MarkerSize', 6, ...
     'MarkerFaceColor', 'b', 'DisplayName', 'Uphill Zone (+10°)');
legend('Location', 'best');

% 4. Axis and color settings
axis(ax_oval, 'equal');
grid(ax_oval, 'on');
title(ax_oval, 'Track Profile (Schematic Oval)', 'FontSize', 14);
xlabel(ax_oval, 'X Distance (m)');
ylabel(ax_oval, 'Y Distance (m)');

colormap(ax_oval, 'jet');
cbar = colorbar(ax_oval);
cbar.Label.String = 'Track Slope (degrees)';
clim([min(color_data) - 1, max(color_data) + 1]);

% --- Subplot 2: Vehicle Parameter Summary Table ---
ax2 = subplot(2, 1, 2);
axis(ax2, 'off');
ylim(ax2, [0 1]);
xlim(ax2, [0 1]);
title(ax2, 'Vehicle Parameters: HUALKANA II', 'FontSize', 14, 'FontWeight', 'bold');

% Format display text
str_title1 = '\bf{Mass and Inertia Parameters:}';
str_M      = sprintf('   Total Mass (M):           %5.2f kg', M);
str_J      = sprintf('   Total Inertia (J_{total}): %5.4f kg\\cdotm^2', J_total);

str_title2 = '\bf{Geometric Parameters:}';
str_rw     = sprintf('   Wheel Radius (r_w):       %5.3f m', r_w);
str_S      = sprintf('   Frontal Area (S):         %5.3f m^2', S);

str_title3 = '\bf{Resistance Parameters:}';
str_Cx     = sprintf('   Drag Coeff. (C_x):        %5.4f', Cx);
str_CdA    = sprintf('   Effective Area (C_dA):    %5.4f m^2', CdA);
str_Crr    = sprintf('   Rolling Coeff. (C_{rr}):  %5.4f', C_rr_base);
str_B      = sprintf('   Viscous Friction (B):     %5.3f Nms/rad', B);

full_text_str = {str_title1, str_M, str_J, ...
                 '', ...
                 str_title2, str_rw, str_S, ...
                 '', ...
                 str_title3, str_Cx, str_CdA, str_Crr, str_B};

text(ax2, 0.05, 0.85, full_text_str, ...
     'VerticalAlignment', 'top', ...
     'FontName', 'Courier New', ...
     'FontSize', 12, ...
     'Interpreter', 'tex');

%% =========================================================================
% 6. DYNAMICS RESULTS VISUALIZATION (FIGURE 1)
% =========================================================================
figure('Name', 'Mechanical Load Simulation (Inverse Dynamics)', ...
       'NumberTitle', 'off', 'WindowState', 'maximized');

% --- Velocity and Track Slope Plot ---
subplot(2, 2, 1);
plot(t_vec_s, v_profile_m_s * 3.6, 'b', 'LineWidth', 2);
title('Desired Velocity Profile vs. Time');
xlabel('Time (s)');
ylabel('Velocity (km/h)');
grid on;
hold on;
yyaxis right;
plot(t_vec_s, rad2deg(alpha_vec), 'r--');
ylabel('Slope (degrees)');
legend('Velocity', 'Slope', 'Location', 'northwest');
hold off;

% --- Vehicle Acceleration Plot ---
subplot(2, 2, 2);
plot(t_vec_s, a_profile_m_s2, 'k', 'LineWidth', 1.5);
title('Vehicle Acceleration vs. Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;

% --- Resistive and Inertial Forces Breakdown ---
subplot(2, 2, 3);
plot(t_vec_s, F_rodadura, 'b');
hold on;
plot(t_vec_s, F_aero, 'g');
plot(t_vec_s, F_pendiente, 'r');
plot(t_vec_s, F_inercia, 'm', 'LineWidth', 1.5);
hold off;
title('Force Components vs. Time');
xlabel('Time (s)');
ylabel('Force (N)');
legend('Rolling Resist.', 'Aero Drag', 'Grade Force', 'Inertial Force (m*a)', ...
       'Location', 'best');
grid on;

% --- Motor Required Load Torque ---
subplot(2, 2, 4);
plot(t_vec_s, T_motor_requerido, 'k', 'LineWidth', 2);
title('Total Required Motor Torque (T_e)');
xlabel('Time (s)');
ylabel('Torque (N\cdot m)');
grid on;
hold on;
% Zero torque baseline (negative values indicate mechanical/regen braking required)
plot(t_vec_s, zeros(size(t_vec_s)), 'r--');
legend('Required T_e', 'Zero Torque Baseline', 'Location', 'best');
hold off;

%% =========================================================================
% HELPER FUNCTION: TRACK PROFILE (SEM BRAZIL)
% =========================================================================

function [alpha, Crr] = get_track_profile(distancia_m, Crr_base)
    % 850 m track circuit profile
    d = mod(distancia_m, 850);
    
    % --- Track Slope (alpha) in radians ---
    if d >= 200 && d < 250         % Turn 1-2 (200 m) - DOWNHILL
        alpha = deg2rad(-3);       % -3 degrees slope
    elseif d >= 600 && d < 650     % Turn 3-4 (200 m) - UPHILL
        alpha = deg2rad(10);       % +10 degrees slope
    else                           % Straightaways
        alpha = 0;
    end
    
    % --- Rolling Resistance Coefficient (Crr) - DIP / SPEED BUMP SIMULATION ---
    Crr = Crr_base;
    if d >= 100 && d < 105         % Dip along first straightaway (5 m length)
        Crr = Crr_base * 5;        % Temporarily increases friction 5x
    end
end