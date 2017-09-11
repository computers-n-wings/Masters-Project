%% SETUP_HELI_3D_CONFIGURATION
%
% SETUP_HELI_3D_CONFIGURATION sets and returns the model parameters 
% of the Quanser 3-DOF Helicopter plant.
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
%%
function [ Kf, mh, mw, mf, mb, Lh, La, Lw, g, F_v1, F_c1, ta] = setup_heli_3d_configuration()

%     % Elevation Friction Model
%     % Propeller force-thrust constant found experimentally (N/V)
%     Kf = 0.115263;
%     % Mass of the helicopter body (kg)
%     mh = 1.308;
%     % Mass of counter-weight (kg)
%     mw = 1.77178;
%     % Mass of front propeller assembly = motor + shield + propeller + body (kg)
%     mf = mh / 2;
%     % Mass of back propeller assembly = motor + shield + propeller + body (kg)
%     mb = mh / 2;
%     % Distance between pitch pivot and each motor (m)
%     Lh  = 0.181658;
%     % Distance between elevation pivot to helicopter body (m)
%     La = 0.632619;
%     % Distance between elevation pivot to counter-weight (m)
%     Lw = 0.469058;
%     % Gravitational Constant (m/s^2)
%     g = 9.81;    
%     % Travel, Pitch, and Elevation Encoder Resolution (rad/count)
%     K_EC_T = 2 * pi / ( 8 * 1024 );
%     K_EC_P = 2 * pi / ( 4 * 1024 );
%     K_EC_E = - 2 * pi / ( 4 * 1024 );
%     % Motor Armature Resistance (Ohm)
%     Rm = 0.83;
%     % Motor Current-Torque Constant (N.m/A)
%     Kt = 0.0182;
%     % Motor Rotor Moment of Inertia (kg.m^2)
%     Jm = 1.91e-6;
%     % Viscous Friction Constant 1  (N.s/rad)
%     F_v1 = -1.85559;
%     % Dynamic Friction Constant 1 (N)
%     F_c1 = -1.5267;
%     % Delay constant
%     ta = -1.38052;

    % Elevation Friction Model
    % Propeller force-thrust constant found experimentally (N/V)
    Kf = 1.1188;
    % Mass of the helicopter body (kg)
    mh = 1.308;
    % Mass of counter-weight (kg)
    mw = 1.924;
    % Mass of front propeller assembly = motor + shield + propeller + body (kg)
    mf = mh / 2;
    % Mass of back propeller assembly = motor + shield + propeller + body (kg)
    mb = mh / 2;
    % Distance between pitch pivot and each motor (m)
    Lh  = 7.0 * 0.0254;
    % Distance between elevation pivot to helicopter body (m)
    La = 26.0 * 0.0254;
    % Distance between elevation pivot to counter-weight (m)
    Lw = 18.5 * 0.0254;
    % Gravitational Constant (m/s^2)
    g = 9.81;    
    % Travel, Pitch, and Elevation Encoder Resolution (rad/count)
    K_EC_T = 2 * pi / ( 8 * 1024 );
    K_EC_P = 2 * pi / ( 4 * 1024 );
    K_EC_E = - 2 * pi / ( 4 * 1024 );
    % Motor Armature Resistance (Ohm)
    Rm = 0.83;
    % Motor Current-Torque Constant (N.m/A)
    Kt = 0.0182;
    % Motor Rotor Moment of Inertia (kg.m^2)
    Jm = 1.91e-6;
    % Viscous Friction Constant 1  (N.s/rad)
%     F_v1 = -0.15;
    F_v1 = -1.4999;
    % Dynamic Friction Constant 1 (N)
%     F_c1 = -0.30;
    F_c1 = -2.1117;
    % Delay constant
%     ta = -1.02359;
    ta = -1.2615;


%     % Frictionless Model 
%     % Propeller force-thrust constant found experimentally (N/V)
%     Kf = 0.1188;
% %     Kf = 0.0167;
%     % Mass of the helicopter body (kg)
%     mh = 1.308;
% %     mh = 1;
%     % Mass of counter-weight (kg)
%     mw = 1.924;
% %     mw = 2.2;
%     % Mass of front propeller assembly = motor + shield + propeller + body (kg)
%     mf = mh / 2;
%     % Mass of back propeller assembly = motor + shield + propeller + body (kg)
%     mb = mh / 2;
%     % Distance between pitch pivot and each motor (m)
%     Lh = 7.0 * 0.0254;
%     % Distance between elevation pivot to helicopter body (m)
%     La = 26.0 * 0.0254;
%     % Distance between elevation pivot to counter-weight (m)
%     Lw = 18.5 * 0.0254;
%     % Gravitational Constant (m/s^2)
%     g = 9.81;    
%     % Travel, Pitch, and Elevation Encoder Resolution (rad/count)
%     K_EC_T = 2 * pi / ( 8 * 1024 );
%     K_EC_P = 2 * pi / ( 4 * 1024 );
%     K_EC_E = - 2 * pi / ( 4 * 1024 );
%     % Motor Armature Resistance (Ohm)
%     Rm = 0.83;
%     % Motor Current-Torque Constant (N.m/A)
%     Kt = 0.0182;
%     % Motor Rotor Moment of Inertia (kg.m^2)
%     Jm = 1.91e-6;


end
%
% end of setup_heli_3d_configuration()