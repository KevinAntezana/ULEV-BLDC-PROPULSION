%% MXUS 48V 250W Hub Motor Parameters.
% --- Electrical ---
V_dc = 48;              % DC bus voltage (V)
R_s = 0.2634;           % Stator resistance per phase (ohm)
L_s = 0.005;            % Stator inductance per phase (H)
P = 20;                 % Number of pole pairs
K_t = 1.36;             % Torque constant (N.m/A)
K_e = K_t/1.5;          % Back-EMF constant (V.s/rad)
lambda = K_t/(1.5*P);   % Flux linkage (Wb)
Pot = 250;              % Rated power (W)       

% --- Mechanical ---
m = 3.7;                % Mass exclugind wheel (kg)
r = 0.25/2;             % Motor radius (m)

%% Motor + Wheel Parameters
m_r = 8.344;            % Mass with wheel (kg)
J_r = 0.2037;           % Moment of Inertia  Ixx (kg.m^2)
r_w = 0.254;            % Wheel radius (m)