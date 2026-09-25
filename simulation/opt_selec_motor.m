%% =========================================================================
% BLDC Hub Motor Sizing and Optimization
% =========================================================================
% This script uses inverse dynamics simulation results to compute optimal
% motor sizing and operating parameters.
%
% PREREQUISITE: Run the track load simulation script first.
% =========================================================================

clc;

if ~exist('T_motor_requerido', 'var') || ~exist('v_profile_m_s', 'var')
    error('Please run the track load simulation script first.');
end

%% =========================================================================
% 1. OPERATING POINT ANALYSIS
% =========================================================================
% Convert linear velocity to angular velocity (RPM and rad/s)
omega_rad_s = v_profile_m_s / r_w;
omega_rpm   = omega_rad_s * (60 / (2 * pi));

% Instantaneous mechanical power (W)
P_mech = T_motor_requerido .* omega_rad_s;

% Separate traction (consumption) and braking (regeneration)
idx_traccion = T_motor_requerido > 0;
idx_frenado  = T_motor_requerido < 0;

T_traccion   = T_motor_requerido(idx_traccion);
w_traccion   = omega_rad_s(idx_traccion);

%% =========================================================================
% 2. CRITICAL SIZING PARAMETERS
% =========================================================================
% --- Torque ---
% Peak Torque: Maximum torque delivered by the motor (uphills/acceleration)
T_peak = max(T_motor_requerido);

% RMS Torque (Nominal): Governs thermal steady-state limits (continuous duty)
% T_rms = sqrt( (1 / T_total) * integral(T^2 dt) )
T_rms = sqrt(trapz(t_vec_s, T_motor_requerido.^2) / t_vec_s(end));

% --- Speed ---
% Maximum required speed on track + safety margin (10%)
RPM_max_req = max(omega_rpm);
RPM_design  = RPM_max_req * 1.10;

% --- Power ---
% Peak mechanical power (used to size battery discharge and power stage)
P_peak_mech = max(P_mech);

% Nominal mechanical power (RMS) estimated for motor continuous rating
P_rms = sqrt(trapz(t_vec_s, P_mech.^2) / t_vec_s(end));

%% =========================================================================
% 3. DC BUS VOLTAGE AND CONSTANTS SELECTION (Kt, Kv)
% =========================================================================
% DC Bus Voltage Selection (Iterative evaluation)
% Standard architectures: 24V, 36V, 48V.
% Select voltage level to keep peak phase current within realistic bounds.
V_options  = [24, 36, 48];
I_peak_est = P_peak_mech ./ V_options;

% Select 48V if current at 36V exceeds 30A; otherwise select 36V
if min(I_peak_est) > 30
    V_bus_opt = 48;
else
    V_bus_opt = 36;
end

% --- Kv Calculation (RPM/V) ---
% Motor must attain RPM_design under available bus voltage.
% Allow a 15% margin for converter voltage drop and control headroom.
% RPM_max_motor = V_bus * Kv * control_headroom(0.85)
Kv_optimo = RPM_design / (V_bus_opt * 0.85);

% --- Kt Calculation (N*m/A) ---
% In SI units: Kt = 1 / Kv_rad_s
% Kv_rad_s = Kv_rpm_v * (2 * pi / 60)
Kv_rad_s  = Kv_optimo * (2 * pi / 60);
Kt_optimo = 1 / Kv_rad_s; % N*m/A (Theoretical ideal)

%% =========================================================================
% 4. EFFICIENCY AND LOSS ESTIMATION (THEORETICAL MODEL)
% =========================================================================
% Estimate winding resistance to assess copper dissipation under load.
% Empirical rule: motors in this power class show ~5-8% voltage drop at I_nominal.
I_nominal_est = T_rms / Kt_optimo;
R_fase_est    = (V_bus_opt * 0.05) / I_nominal_est; % Estimated phase resistance

% Recalculate theoretical lap efficiency using sized parameters
I_inst      = abs(T_motor_requerido) / Kt_optimo;  % Phase current demand
P_cobre     = I_inst.^2 * R_fase_est;              % Joule losses (I^2 * R)
P_elec_in   = P_mech + P_cobre;                    % Total electrical input power

% Instantaneous efficiency (evaluated only during positive traction)
eff_inst               = zeros(size(P_mech));
eff_inst(idx_traccion) = P_mech(idx_traccion) ./ P_elec_in(idx_traccion);
eff_inst(eff_inst > 1 | eff_inst < 0) = 0;         % Data cleanup

% Average cycle efficiency
Energy_out = trapz(t_vec_s(idx_traccion), P_mech(idx_traccion));
Energy_in  = trapz(t_vec_s(idx_traccion), P_elec_in(idx_traccion));
Eff_cycle  = (Energy_out / Energy_in) * 100;

%% =========================================================================
% 5. RESULTS VISUALIZATION
% =========================================================================
figure('Name', 'Optimal Motor Sizing and Operating Limits', ...
       'NumberTitle', 'off', 'WindowState', 'maximized');

% --- Torque vs. Speed Operating Map ---
subplot(2, 2, 1);
scatter(omega_rpm, abs(T_motor_requerido), 10, P_mech, 'filled');
hold on;
% Safe Operating Area (SOA) boundaries
rectangle('Position', [0, 0, RPM_design, T_rms], ...
          'EdgeColor', 'g', 'LineWidth', 2, 'LineStyle', '--');
line([0, RPM_design], [T_peak, T_peak], 'Color', 'r', 'LineWidth', 2);
line([RPM_design, RPM_design], [0, T_peak], 'Color', 'r', 'LineWidth', 2);
colormap('jet');
c = colorbar;
c.Label.String = 'Mechanical Power (W)';
title('Operating Map: Torque vs. Speed');
xlabel('Speed (RPM)');
ylabel('Torque (N\cdot m)');
legend('Operating Points', 'Continuous Zone (RMS)', 'Peak Limits', 'Location', 'northeast');
grid on;
xlim([0, RPM_design * 1.1]);
ylim([0, T_peak * 1.2]);

% --- Theoretical Efficiency Histogram ---
subplot(2, 2, 2);
histogram(eff_inst(idx_traccion) * 100, 20, 'FaceColor', 'b');
title(sprintf('Estimated Motor Efficiency (Mean: %.1f%%)', Eff_cycle));
xlabel('Efficiency (%)');
ylabel('Sample Count');
grid on;

% --- Torque and Power vs. Time ---
subplot(2, 2, 3);
plot(t_vec_s, T_motor_requerido, 'k', 'LineWidth', 1.5);
hold on;
yline(T_rms, 'g--', 'RMS Limit', 'LineWidth', 1.5);
yline(T_peak, 'r--', 'Peak Limit', 'LineWidth', 1.5);
title('Torque Demand vs. Time with Sizing Boundaries');
xlabel('Time (s)');
ylabel('Torque (N\cdot m)');
legend('Required Torque', 'RMS Rating', 'Peak Limit', 'Location', 'best');
grid on;

% --- Sizing Specification Summary ---
subplot(2, 2, 4);
axis off;
text(0, 1.00, 'OPTIMAL MOTOR SPECIFICATIONS:', 'FontWeight', 'bold', 'FontSize', 12);
text(0, 0.85, sprintf('Suggested Bus Voltage:  %d V DC', V_bus_opt));
text(0, 0.75, sprintf('Suggested KV Rating:    %.2f RPM/V', Kv_optimo));
text(0, 0.65, sprintf('Torque Constant (Kt):   %.4f N\\cdot m/A', Kt_optimo));
text(0, 0.55, '----------------------------------------');
text(0, 0.45, sprintf('Nominal Power (RMS):    %.2f W', P_rms));
text(0, 0.35, sprintf('Peak Mechanical Power:  %.2f W', P_peak_mech));
text(0, 0.25, '----------------------------------------');
text(0, 0.15, sprintf('Nominal Torque (RMS):   %.2f N\\cdot m', T_rms));
text(0, 0.05, sprintf('Peak Torque:            %.2f N\\cdot m', T_peak));
text(0, -0.05, sprintf('Max Design Speed:       %.0f RPM', RPM_design));

%% =========================================================================
% 6. CONSOLE REPORT
% =========================================================================
fprintf('\n=== MOTOR SIZING AND SELECTION REPORT ===\n');
fprintf('Based on simulated driving cycle dynamics:\n');
fprintf('1. System DC Voltage:   %d V DC\n', V_bus_opt);
fprintf('2. Target Motor KV:     %.2f RPM/V (Target range: %.0f - %.0f RPM/V)\n', ...
        Kv_optimo, Kv_optimo * 0.9, Kv_optimo * 1.1);
fprintf('3. Continuous Torque:   %.2f N*m (Thermal steady-state continuous limit)\n', T_rms);
fprintf('4. Peak Torque:         %.2f N*m (Required for gradients and acceleration)\n', T_peak);
fprintf('5. Mechanical Power:    %.2f W (Nominal RMS) / %.2f W (Peak)\n', P_rms, P_peak_mech);
fprintf('6. Cycle Efficiency:    %.2f %% (Estimated average under sizing model)\n', Eff_cycle);
fprintf('=========================================\n');