%% SETUP_LAB_HELI_3D
%
% This script sets the model parameters and designs a PID position
% controller using LQR for the Quanser 3-DOF Helicopter plant.
%
clear all;
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/lib
%
%% User defined 3DOF helicopter system configuration
% Amplifier Gain used for yaw and pitch axes: set VoltPAQ to 3.
K_AMP = 3;
% Amplifier Maximum Output Voltage (V)
VMAX_AMP = 24;
% Digital-to-Analog Maximum Voltage (V): set to 10 for Q4/Q8 cards
VMAX_DAC = 10;
% Initial elvation angle (rad)
elev_0 = -27.5*pi/180;

%% User defined Filter design
% Specifications of a second-order low-pass filter
wcf = 2 * pi * 20; % filter cutting frequency
zetaf = 0.9;        % filter damping ratio
% Anti-windup: integrator saturation (V)
SAT_INT_ERR_ELEV = 7.5;
SAT_INT_ERR_TRAVEL = 7.5;

%% User defined command settings
% Note: These limits are imposed on both the program and joystick commands.
% Elevation position command limit (deg)
CMD_ELEV_POS_LIMIT_LOWER = elev_0*180/pi;
CMD_ELEV_POS_LIMIT_UPPER = -CMD_ELEV_POS_LIMIT_LOWER;
% Maximum Rate of Desired Position (rad/s)
CMD_RATE_LIMIT = 45.0 * pi / 180;

%% System Modelling
% These parameters are used for model representation and controller design.
[ Kf, mh, mw, mf, mb, Lh, La, Lw, g, F_v1, F_c1, ta] = setup_heli_3d_configuration();

c1 = La*Kf/(mw*Lw^2+2*mf*La^2);
c2 = 0.5*Kf/mf/Lh;
c3 = (2*mf*La-mw*Lw)*g/(2*mf*La^2+2*mf*Lh^2+mw*Lw^2);

% For the following state vector: X = [ elevation; pitch; travel, elev_dot, pitch_dot, travel_dot]
% Initialization the state-Space representation of the open-loop System
A=zeros(6,6);
A( 1, 4 ) = 1;
A( 2, 5 ) = 1;
A( 3, 6 ) = 1;
A( 6, 2 ) = c3;

B=zeros(6,2);
B( 4, 1 ) = c1;
B( 4, 2 ) = c1;
B( 5, 1 ) = c2;
B( 5, 2 ) = -c2;

C = eye(6);

D = zeros(6,2);

% Augment state: Xi = [ elevation; pitch; travel, elev_dot, pitch_dot, travel_dot, elev_int, travel_int]
Ai = A;
Ai(7,1) = 1; % elevation integrator 
Ai(8,3) = 1; % travel integrator 
Ai(8,8) = 0;
Bi = B;
Bi(8,2) = 0;

%% LQR-PID Controller design
%Weights:
Q = diag([100 1 10 0 0 2 10 0.1]);
% Q = eye(8);

R = 0.05*diag([1 1]);
% Automatically calculate the LQR controller gain
K = lqr( Ai, Bi, Q, R );

% K = [37.67,13.25,-11.50,20.96,4.78,-16.08,10.00,-1.00; ...
%     37.67,-13.25,11.50,20.96,-4.78,16.08,10.00,1.00];

% Display the calculated gains
% disp( ' ' )
% disp( 'Calculated LQR controller gain elements: ' )
% K 

%% Non-linear model parameters (for closed-loop simulation only):
%Modified on 21/11/2016

%System parameters
par = [Kf;mh;mw;mf;mb;Lh;La;Lw;g;F_v1;F_c1;ta]; 

%Initial condition
X0=[elev_0;0;0;0;0;0;0;0];
%% Feed-forward input:

Vop=0.5*g*(Lw*mw-2*La*mf)/(La*Kf);

%% Actuator Dynamics Model
Ahat = zeros(8);
Ahat(1,4) = 1;
Ahat(2,5) = 1;
Ahat(3,6) = 1;
Ahat(4,7) = c1;
Ahat(4,8) = c1;
Ahat(5,7) = c2;
Ahat(5,8) = -c2;
Ahat(6,2) = c3;
Ahat(7,7) = ta;
Ahat(8,8) = ta;

Bhat = zeros(8,2);
Bhat(7,1) = -ta;
Bhat(8,2) = -ta;

Chat = eye(8);

Dhat = zeros(8,2);

% Augmented state: Xhati = [ elevation; pitch; travel, elev_dot, pitch_dot,
%                  travel_dot, input1_delay, input2_delay, elev_int, travel_int]
Aihat = Ahat;
Aihat(9,1) = 1; % elevation integrator 
Aihat(10,3) = 1; % travel integrator 
Aihat(10,10) = 0;

Bihat = Bhat;
Bihat(10,2) = 0;

%% LQR-PID Controller design
%Weights:
Qhat = diag([100 1 10 0 0 2 0 0 10 0.1]);

Rhat = 0.05*diag([1 1]);
% Automatically calculate the LQR controller gain
Khat = lqr(Aihat, Bihat, Qhat, Rhat );




