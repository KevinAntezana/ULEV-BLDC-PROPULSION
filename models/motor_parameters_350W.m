%% 350W Motor parameters (BASELINE)
% --- Electrical ---
V_dc = 42;              % DC bus voltage (V)
R_s = 0.332;            % Stator resistance per phase (ohm)
L_s = 0.0050184;        % Stator inductance per phase (H)
P = 20;                 % Number of pole pairs
K_e = 1.0308;           % Back-EMF constant (V·s/rad)

lambda = K_e / 1.0308;  % Flux linkage (Wb)
Pot = 350;              % Rated power (W)        

% --- Mechanical ---
m = 4.136;              % Mass exclugind wheel (kg)
r = 0.2037/2;           % Motor radius (m)

%% Motor + Wheel Parameters     
m_r = 8.78;             % Mass with wheel (kg)
J_r = 0.2037;           % Moment of Inertia  Ixx (kg.m^2)
r_w = 0.254;            % Wheel radius (m)