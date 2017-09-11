function [sys,x0,str,ts] = Nonlinear_Helicopter_SFunc(t,x,u,flag,varargin)
% S-Function style nonlinear model for the lab helicopter setup
% Equations based on Quanser document 'Dynamic Equations for the 3-DOF Helicopter'
% Version 1 (21/11/2016)


persistent DERX 

switch flag,
  % Initialization 
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(varargin);

  % Derivatives 
  case 1,
   sys = DERX;
   
  % Update 
  case 2,
    sys=[];

  % Outputs 
  case 3,
   [sys,DERX] = NonlinearHelicopter(t,x,u,varargin);
   

  % GetTimeOfNextVarHit 
  case 4,
    sys=[];

  % Terminate 
  case 9,
    sys=[];

  % Unexpected flags 
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end


function [sys,x0,str,ts]=mdlInitializeSizes(varargin)

sizes = simsizes;

sizes.NumContStates  = 8;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 8;
sizes.NumInputs      = 2;
sizes.DirFeedthrough =  1;
sizes.NumSampleTimes =  1;   % at least one sample time is needed

sys = simsizes(sizes);

% initialize the initial conditions

%user supplied IC
if nargin > 0,
  x0  = varargin{1}{2};
else % default IC
  x0  = [-27.5*pi/180;0;0;0;0;0;0;0];
end;

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];

function [Y,DERX] = NonlinearHelicopter(t,X,U,varargin)

% % Setup for Friction model 1
% [ Kf, m_h, mw, mf, mb, Lh, La, Lw, g, F_v, F_c] ...
%     = setup_heli_3d_configuration();

addpath /home/jcg216/Work/MSC_Project/Matlab_Files/lib

% Setup for Friction model 2
[ Kf, mh, mw, mf, mb, Lh, La, Lw, g, F_v1, F_c1, ta] = setup_heli_3d_configuration();

%Nonlinear System
epsilon_dot=X(4);
rho_dot=X(5);
lambda_dot=X(6);

% Elevation Friction Model
epsilon_ddot = ((-0.4e1 * (cos(X(2)) ^ 2 - 0.2e1) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * (mb - mf) * La * sin(X(2)) * X(6) ^ 2 * cos(X(1)) ^ 3 + (0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * Lh * (mb - mf) * La * X(6) * cos(X(2)) ^ 3 + (-0.8e1 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * X(6) ^ 2 * sin(X(1)) + ((-0.8e1 * X(5) * X(6) * Lh ^ 2 * mf + Lw * mw * g) * mb ^ 2 + (-0.8e1 * Lh ^ 2 * mf ^ 2 * X(5) * X(6) - 0.2e1 * Lw * g * mf * mw) * mb + g * Lw * mf ^ 2 * mw) * La ^ 2 - 0.4e1 * ((Lh ^ 2 * mf - Lw ^ 2 * mw / 0.4e1) * mb ^ 2 + mf * (Lh ^ 2 * mf + Lw ^ 2 * mw / 0.2e1) * mb - Lw ^ 2 * mf ^ 2 * mw / 0.4e1) * g * La + Lh ^ 2 * Lw * mw * (mf + mb) ^ 2 * (-0.2e1 * Lw * X(5) * X(6) + g)) * cos(X(2)) ^ 2 - 0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * Lh * (mb - mf) * La * X(6) * cos(X(2)) - 0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (X(6) ^ 2 * ((mf + mb) * La ^ 2 - Lh ^ 2 * mb - Lh ^ 2 * mf + Lw ^ 2 * mw) * sin(X(1)) + La * g * (mf + mb) - 0.2e1 * X(5) * X(6) * Lh ^ 2 * mb - 0.2e1 * X(5) * X(6) * Lh ^ 2 * mf - Lw * mw * g)) * cos(X(1)) ^ 2 + ((0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) ^ 2 * Lh * (mb - mf) * La * sin(X(2)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g + F_v1 * X(4))) * cos(X(2)) ^ 2 + (-0.8e1 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (X(6) * sin(X(1)) + X(5)) * X(4) * Lh ^ 2 * sin(X(2)) - ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) * (X(8) - X(7)) * (mb - mf) * La * Kf) * cos(X(2)) - 0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (Lh * (mb - mf) * ((0.2e1 * X(5) * X(6) * La + g) * sin(X(1)) + La * (X(5) ^ 2 + X(6) ^ 2)) * sin(X(2)) + (-g * mb - g * mf - Kf * (X(8) + X(7))) * La + Lw * mw * g - F_v1 * X(4))) * cos(X(1)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * cos(X(2)) * (((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g) * cos(X(2)) ^ 2 - Kf * sin(X(1)) * Lh * (X(8) - X(7)) * sin(X(2)) + (-g * mb - g * mf - Kf * (X(8) + X(7))) * La + Lw * mw * g)) * cosh(0.10e2 * X(1)) + F_c1 * cos(X(1)) * sinh(0.10e2 * X(1)) * (((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * cos(X(2)) ^ 2 + 0.4e1 * La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb))) / (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) / cosh(0.10e2 * X(1)) / ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) / cos(X(1)) / 0.4e1;
rho_ddot = ((-0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * sin(X(2)) * X(6) ^ 2 * ((-mb - mf) * Lh ^ 2 + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * cos(X(2)) * cos(X(1)) ^ 4 + (0.4e1 * sin(X(1)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * (mb - mf) * La * X(6) ^ 2 * cos(X(2)) ^ 2 + 0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * X(4) * Lh * X(6) * cos(X(2)) + (mb - mf) * (-0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * La * X(6) ^ 2 * sin(X(1)) - 0.4e1 * (0.2e1 * X(5) * X(6) * La ^ 2 * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * X(5) * X(6) + g) / 0.4e1) * La * Lh ^ 2 + ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * Lw * (La + Lw) * g * mw)) * cos(X(2)) * cos(X(1)) ^ 3 + (-Lh ^ 2 * (-0.8e1 * sin(X(1)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * (mb - mf) * La * X(6) * sin(X(2)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * (X(8) - X(7)) * Kf) * cos(X(2)) ^ 2 + (((-0.4e1 * (mf + mb) * (0.2e1 * X(5) * X(6) * La ^ 2 * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * X(5) * X(6) + g) / 0.4e1) * Lh ^ 2 + g * La * Lw * mw * (mb - mf) ^ 2 * (La + Lw)) * sin(X(1)) + 0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * ((X(4) ^ 2 - 0.2e1 * X(6) ^ 2) * (mf + mb) * Lh ^ 2 + ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * X(4) ^ 2)) * Lh * sin(X(2)) + (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (mb - mf) * La * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g + F_v1 * X(4))) * cos(X(2)) - (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (X(8) - X(7)) * ((-mb - mf) * Lh ^ 2 + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * Kf) * cos(X(1)) ^ 2 + (-0.4e1 * sin(X(1)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) ^ 2 * Lh ^ 2 * (mb - mf) * La * cos(X(2)) ^ 3 + (0.8e1 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * Lh ^ 3 * X(5) * sin(X(1)) + 0.8e1 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * X(6) * Lh ^ 3 + (mf + mb) * (mb - mf) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g) * La * Lh ^ 2 + ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (mb - mf) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g) * La) * cos(X(2)) ^ 2 + Lh * (sin(X(1)) * ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g + F_v1 * X(4)) * sin(X(2)) + 0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * (mb - mf) * (La * (X(5) ^ 2 + X(6) ^ 2) * sin(X(1)) + 0.2e1 * X(5) * X(6) * La + g)) * cos(X(2)) - (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (0.2e1 * Kf * sin(X(1)) * La * Lh * (mb - mf) * (X(8) - X(7)) * sin(X(2)) + 0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * X(6) * Lh + (mb - mf) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g) * La)) * cos(X(1)) + Lh * (sin(X(1)) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g) * sin(X(2)) + Kf * Lh * (X(8) - X(7))) * (((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * cos(X(2)) ^ 2 - (mf + mb) * (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw))) * cosh(0.10e2 * X(1)) + F_c1 * ((Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (mb - mf) * La * cos(X(1)) + sin(X(1)) * ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * Lh * sin(X(2))) * cos(X(1)) * sinh(0.10e2 * X(1)) * cos(X(2))) / (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) / cosh(0.10e2 * X(1)) / Lh / (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) / cos(X(1)) ^ 2 / 0.4e1;
lambda_ddot = ((0.4e1 * (X(6) * cos(X(1)) + X(4)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * cos(X(1)) * (mb - mf) * La * (-X(6) * cos(X(1)) + X(4)) * cos(X(2)) ^ 3 + ((-0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * Lh * (mb - mf) * La * X(6) * cos(X(1)) ^ 2 - ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g)) * sin(X(2)) - Lh * (0.8e1 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (X(6) * sin(X(1)) + X(5)) * X(4) * Lh * cos(X(1)) + sin(X(1)) * ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * (X(8) - X(7)) * Kf)) * cos(X(2)) ^ 2 - cos(X(1)) * (((-0.8e1 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * X(6) ^ 2 * sin(X(1)) + ((-0.8e1 * X(5) * X(6) * Lh ^ 2 * mf + Lw * mw * g) * mb ^ 2 + (-0.8e1 * Lh ^ 2 * mf ^ 2 * X(5) * X(6) - 0.2e1 * Lw * g * mf * mw) * mb + g * Lw * mf ^ 2 * mw) * La ^ 2 - 0.4e1 * ((Lh ^ 2 * mf - Lw ^ 2 * mw / 0.4e1) * mb ^ 2 + mf * (Lh ^ 2 * mf + Lw ^ 2 * mw / 0.2e1) * mb - Lw ^ 2 * mf ^ 2 * mw / 0.4e1) * g * La + Lh ^ 2 * Lw * mw * (mf + mb) ^ 2 * (-0.2e1 * Lw * X(5) * X(6) + g)) * cos(X(1)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g + F_v1 * X(4))) * sin(X(2)) + 0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (-0.2e1 * X(6) ^ 2 * La * cos(X(1)) ^ 2 + (0.2e1 * X(5) * X(6) * La + g) * sin(X(1)) + La * (X(5) ^ 2 + X(6) ^ 2)) * Lh * (mb - mf)) * cos(X(2)) + ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) * ((Kf * La * (mb - mf) * (X(8) - X(7)) * cos(X(1)) + (mf + mb) * ((g * mb + g * mf + Kf * (X(8) + X(7))) * La - Lw * mw * g)) * sin(X(2)) + sin(X(1)) * (0.8e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(4) * X(6) * cos(X(1)) + Kf * Lh * (mf + mb) * (X(8) - X(7))))) * cosh(0.10e2 * X(1)) - ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * F_c1 * cos(X(1)) * sin(X(2)) * sinh(0.10e2 * X(1)) * cos(X(2))) / (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) / cosh(0.10e2 * X(1)) / ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) / cos(X(1)) ^ 2 / 0.4e1;
U1_delay = ta*X(7) - ta*U(1);
U2_delay = ta*X(8) - ta*U(2);

% epsilon_ddot = (-0.4e1 * X(6) ^ 2 * (cos(X(2)) ^ 2 - 0.2e1) * La * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * sin(X(2)) * Lh * cos(X(1)) ^ 3 + (0.8e1 * X(6) * X(4) * La * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * Lh * cos(X(2)) ^ 3 + (-0.8e1 * X(6) ^ 2 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * sin(X(1)) + ((-0.8e1 * X(5) * X(6) * Lh ^ 2 * mf + Lw * mw * g) * mb ^ 2 + (-0.8e1 * Lh ^ 2 * mf ^ 2 * X(5) * X(6) - 0.2e1 * Lw * g * mf * mw) * mb + g * Lw * mf ^ 2 * mw) * La ^ 2 - 0.4e1 * ((Lh ^ 2 * mf - Lw ^ 2 * mw / 0.4e1) * mb ^ 2 + mf * (Lh ^ 2 * mf + Lw ^ 2 * mw / 0.2e1) * mb - Lw ^ 2 * mf ^ 2 * mw / 0.4e1) * g * La + Lh ^ 2 * Lw * mw * (mf + mb) ^ 2 * (-0.2e1 * Lw * X(5) * X(6) + g)) * cos(X(2)) ^ 2 - 0.8e1 * X(6) * X(4) * La * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * Lh * cos(X(2)) - 0.4e1 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (X(6) ^ 2 * ((mf + mb) * La ^ 2 - Lh ^ 2 * mb - Lh ^ 2 * mf + Lw ^ 2 * mw) * sin(X(1)) + La * g * (mf + mb) - 0.2e1 * X(5) * X(6) * Lh ^ 2 * mb - 0.2e1 * X(5) * X(6) * Lh ^ 2 * mf - Lw * mw * g)) * cos(X(1)) ^ 2 + ((0.4e1 * X(4) ^ 2 * La * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * Lh * sin(X(2)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((F_c1 * sign(X(1))) + (g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g + F_v1 * X(4))) * cos(X(2)) ^ 2 + (-0.8e1 * (mf + mb) * X(4) * (X(6) * sin(X(1)) + X(5)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * sin(X(2)) + (X(7) - X(8)) * ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) * Kf * La * (mb - mf)) * cos(X(2)) - 0.4e1 * (((0.2e1 * X(5) * X(6) * La + g) * sin(X(1)) + La * (X(5) ^ 2 + X(6) ^ 2)) * (mb - mf) * Lh * sin(X(2)) - (F_c1 * sign(X(1))) + (-g * mb - g * mf - Kf * (X(7) + X(8))) * La + Lw * mw * g - F_v1 * X(4)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1)) * cos(X(1)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * cos(X(2)) * (((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g) * cos(X(2)) ^ 2 + Kf * sin(X(1)) * Lh * (X(7) - X(8)) * sin(X(2)) + (-g * mb - g * mf - Kf * (X(7) + X(8))) * La + Lw * mw * g)) / ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) / (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) / cos(X(1)) / 0.4e1;
% rho_ddot = (-0.4e1 * X(6) ^ 2 * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * ((-mb - mf) * Lh ^ 2 + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * sin(X(2)) * cos(X(2)) * Lh * cos(X(1)) ^ 4 + (0.4e1 * X(6) ^ 2 * La * sin(X(1)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * Lh ^ 2 * cos(X(2)) ^ 2 + 0.8e1 * X(6) * X(4) * ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * cos(X(2)) + (-0.8e1 * X(6) ^ 2 * La * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * sin(X(1)) - 0.4e1 * (0.2e1 * X(5) * X(6) * La ^ 2 * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * X(5) * X(6) + g) / 0.4e1) * La * Lh ^ 2 + Lw * ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * g * mw * (La + Lw)) * (mb - mf)) * cos(X(2)) * cos(X(1)) ^ 3 + ((0.8e1 * X(6) * X(4) * La * sin(X(1)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * sin(X(2)) + (X(7) - X(8)) * Kf * ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2)) * Lh ^ 2 * cos(X(2)) ^ 2 + (((-0.4e1 * (mf + mb) * (0.2e1 * X(5) * X(6) * La ^ 2 * mb * mf + g * La * mb * mf - Lw * mw * (mf + mb) * (-0.2e1 * Lw * X(5) * X(6) + g) / 0.4e1) * Lh ^ 2 + g * La * Lw * mw * (mb - mf) ^ 2 * (La + Lw)) * sin(X(1)) + 0.4e1 * ((X(4) ^ 2 - 0.2e1 * X(6) ^ 2) * (mf + mb) * Lh ^ 2 + X(4) ^ 2 * ((mf + mb) * La ^ 2 + Lw ^ 2 * mw)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1)) * Lh * sin(X(2)) + (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * La * ((F_c1 * sign(X(1))) + (g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g + F_v1 * X(4)) * (mb - mf)) * cos(X(2)) + (X(7) - X(8)) * (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * Kf * ((-mb - mf) * Lh ^ 2 + (mf + mb) * La ^ 2 + Lw ^ 2 * mw)) * cos(X(1)) ^ 2 + (-0.4e1 * X(4) ^ 2 * La * sin(X(1)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * Lh ^ 2 * cos(X(2)) ^ 3 + (0.8e1 * (mf + mb) * X(4) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * X(5) * Lh ^ 3 * sin(X(1)) + 0.8e1 * X(6) * (mf + mb) * X(4) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 3 + (mf + mb) * La * (mb - mf) * ((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g) * Lh ^ 2 + ((mf + mb) * La ^ 2 + Lw ^ 2 * mw) * La * (mb - mf) * ((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g)) * cos(X(2)) ^ 2 + (((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * sin(X(1)) * ((F_c1 * sign(X(1))) + (g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g + F_v1 * X(4)) * sin(X(2)) + 0.4e1 * (La * (X(5) ^ 2 + X(6) ^ 2) * sin(X(1)) + 0.2e1 * X(5) * X(6) * La + g) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (mb - mf) * Lh) * Lh * cos(X(2)) - (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (-0.2e1 * Kf * sin(X(1)) * La * Lh * (mb - mf) * (X(7) - X(8)) * sin(X(2)) + 0.8e1 * X(6) * X(4) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh + La * (mb - mf) * ((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g))) * cos(X(1)) + (sin(X(1)) * ((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g) * sin(X(2)) - Kf * (X(7) - X(8)) * Lh) * (((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * cos(X(2)) ^ 2 - (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) * (mf + mb)) * Lh) / (Lh ^ 2 * (mf + mb) + (mf + mb) * La ^ 2 + Lw ^ 2 * mw) / (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) / cos(X(1)) ^ 2 / Lh / 0.4e1;
% lambda_ddot = (0.4e1 * La * cos(X(1)) * (mb - mf) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * (X(6) * cos(X(1)) + X(4)) * (-X(6) * cos(X(1)) + X(4)) * Lh * cos(X(2)) ^ 3 + ((-0.8e1 * X(6) * X(4) * La * (mb - mf) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * cos(X(1)) ^ 2 - ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g)) * sin(X(2)) + (-0.8e1 * (mf + mb) * X(4) * (X(6) * sin(X(1)) + X(5)) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh * cos(X(1)) + (X(7) - X(8)) * Kf * ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * sin(X(1))) * Lh) * cos(X(2)) ^ 2 - (((-0.8e1 * X(6) ^ 2 * (mf + mb) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh ^ 2 * sin(X(1)) + ((-0.8e1 * X(5) * X(6) * Lh ^ 2 * mf + Lw * mw * g) * mb ^ 2 + (-0.8e1 * Lh ^ 2 * mf ^ 2 * X(5) * X(6) - 0.2e1 * Lw * g * mf * mw) * mb + g * Lw * mf ^ 2 * mw) * La ^ 2 - 0.4e1 * ((Lh ^ 2 * mf - Lw ^ 2 * mw / 0.4e1) * mb ^ 2 + mf * (Lh ^ 2 * mf + Lw ^ 2 * mw / 0.2e1) * mb - Lw ^ 2 * mf ^ 2 * mw / 0.4e1) * g * La + Lh ^ 2 * Lw * mw * (mf + mb) ^ 2 * (-0.2e1 * Lw * X(5) * X(6) + g)) * cos(X(1)) + ((mb - mf) ^ 2 * La ^ 2 + Lh ^ 2 * (mf + mb) ^ 2) * ((F_c1 * sign(X(1))) + (g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g + F_v1 * X(4))) * sin(X(2)) + 0.4e1 * (-0.2e1 * X(6) ^ 2 * La * cos(X(1)) ^ 2 + (0.2e1 * X(5) * X(6) * La + g) * sin(X(1)) + La * (X(5) ^ 2 + X(6) ^ 2)) * (mb - mf) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * Lh) * cos(X(1)) * cos(X(2)) + ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) * ((-Kf * La * (mb - mf) * (X(7) - X(8)) * cos(X(1)) + (mf + mb) * ((g * mb + g * mf + Kf * (X(7) + X(8))) * La - Lw * mw * g)) * sin(X(2)) - sin(X(1)) * (-0.8e1 * X(6) * X(4) * (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) * cos(X(1)) + Kf * Lh * (mf + mb) * (X(7) - X(8))))) / ((mf + mb) * La ^ 2 + Lh ^ 2 * mb + Lh ^ 2 * mf + Lw ^ 2 * mw) / cos(X(1)) ^ 2 / (La ^ 2 * mb * mf + Lw ^ 2 * mw * (mf + mb) / 0.4e1) / 0.4e1;
% U1_delay = ta*X(7) - ta*U(1);
% U2_delay = ta*X(8) - ta*U(2);

%State Derivative
DERX=[epsilon_dot;rho_dot;lambda_dot;epsilon_ddot;rho_ddot;lambda_ddot;U1_delay; U2_delay];

%Output
Y=[X(1);X(2);X(3);epsilon_dot;rho_dot;lambda_dot;X(7);X(8)];
