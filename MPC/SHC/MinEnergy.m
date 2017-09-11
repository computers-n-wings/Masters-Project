function [problem,guess] = Nonregulation
% Global_Ref_Trajectory - Determine the reference trajectory for a minimum
% travel time of MPC control of a labratory helicopter 
%
% Syntax:  [problem,guess] = Global_Ref_Trajectory
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% Subfunctions: L (stageCost), 
%		E (boundaryCost), 
%		f (ODE right-hand side), 
%		g (path constraints), 
%		b (boundary constraints)
% MAT-files required: none
%
% Copyright (C) 2010 Paola Falugi, Eric Kerrigan and Eugene van Wyk. All Rights Reserved.
% This code is published under the BSD License.
% Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) 5 May 2010
% iclocs@imperial.ac.uk

% Modified by James Gross
% Date: 09/06/2017

%------------- BEGIN CODE --------------

addpath /home/jcg216/Work/MSC_Project/Matlab_Files/lib

% Global parameters
[ Kf, mh, mw, mf, mb, Lh, La, Lw, grav, F_v1, F_c1] = setup_heli_3d_configuration();
ta = -1.2615;

% Constraint values

% epsmax = 27.5 * pi / 180;
% rhomax = 90 * pi / 180;

epsmax = 30 * pi / 180;
rhomax = 92.5 * pi / 180;

epsdotmax = Inf;
rhodotmax = Inf;
lamdotmax = Inf;
Vmax = 24;

% Boundary values
x_initial = [0 0 0 0 0 0 0 0];
x_final = [0 0 pi 0 0 0 2 -17];

% Initial time. t0<tf
problem.time.t0 = 0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min = 100;     
problem.time.tf_max = 100;
guess.tf = 100;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl = [];
problem.parameters.pu = [];
guess.parameters = [];

% Initial conditions for system
problem.states.x0 = x_initial;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l = x_initial; 
problem.states.x0u = x_initial; 

% State bounds. xl=< x <=xu
problem.states.xl = [-epsmax -rhomax -Inf -epsdotmax -rhodotmax -lamdotmax -Vmax -Vmax];
problem.states.xu = [epsmax rhomax Inf epsdotmax rhodotmax lamdotmax Vmax Vmax];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl = [0 0 pi 0 0 0 -Vmax -Vmax];
problem.states.xfu = [0 0 pi 0 0 0 Vmax Vmax];

% Guess the state trajectories with [x0 xf]
guess.states(1,:) = x_initial';
guess.states(2,:) = x_final';

% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N = 0;

% Input bounds
problem.inputs.ul = [-Vmax -Vmax];
problem.inputs.uu = [Vmax Vmax];

% Guess the input sequences with [u0 uf]
guess.inputs(:,1) = [1 1];
guess.inputs(:,2) = [1 -1];

% Choose the set-points if required
problem.setpoints.states = [];
problem.setpoints.inputs = [];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.gl = [];
problem.constraints.gu = [];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl = [];
problem.constraints.bu = [];

% Store the necessary problem parameters used in the functions
problem.data.Lh = Lh;
problem.data.mf = mf;
problem.data.mb = mb;
problem.data.mw = mw;
problem.data.La = La;
problem.data.Lw = Lw;
problem.data.grav = grav;
problem.data.Kf = Kf;
problem.data.F_c1 = F_c1;
problem.data.F_v1 = F_v1;
problem.data.ta = ta;


% Get function handles and return to Main.m
problem.functions = {@L,@E,@f,@g,@b};

%------------- END OF CODE --------------

function stageCost = L(x,xr,u,ur,p,t,data)

% L - Returns the stage cost.
% The function must be vectorised and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorised stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimisation. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------
%% Hard Constraints
penalty = 0*t;

% %% Penalty Function
% k = 10000;
% delta = 0.000001;
% pitchmax = pi/2;
% elevmax = 27.5*pi/180;
% rho = 10;
% 
% elev_pen = log(1+exp(k*(sqrt(x(:,1).^2+delta)-elevmax)))/k;
% pit_pen = log(1+exp(k*(sqrt(x(:,2).^2+delta)-pitchmax)))/k;
% penalty = rho*(elev_pen + pit_pen);

%% Min Energy Stage Cost
u1 = u(:,1); u2 = u(:,2);

stageCost=u1.^2+u2.^2 + penalty;

%------------- END OF CODE --------------


function boundaryCost = E(x0,xf,u0,uf,p,tf,data) 


% E - Returns the boundary value cost
%
% Syntax:  boundaryCost=E(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost = 0;

%------------- END OF CODE --------------


function dx = f(x,u,p,t,data)

% f - Returns the ODE right hand side where x'= f(x,u,p,t)
% The function must be vectorised and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% state corresponds to one column of dx.
% 
% 
% Syntax:  dx = f(x,u,p,t,data)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    data-structured variable containing the values of additional data used inside
%          the function 
%
% Output:
%    dx - time derivative of x
%
%  Remark: If the i-th ODE right hand side does not depend on variables it is necessary to multiply
%          the assigned value by a vector of ones with the same length  of t  in order 
%          to have  a vector with the right dimesion  when called for the optimisation. 
%          Example: dx(:,i)= 0*ones(size(t,1)); 
%
%------------- BEGIN CODE --------------

Lh = data.Lh; mf = data.mf; mb = data.mb; mw = data.mw;
Kf = data.Kf; grav = data.grav; La = data.La; Lw = data.Lw;
F_v1 = data.F_v1; 
F_c1 = data.F_c1; 
ta = data.ta;

%Define states
x1 = x(:,1);
x2 = x(:,2);
x3 = x(:,3);
x4 = x(:,4);
x5 = x(:,5);
x6 = x(:,6);
x7 = x(:,7);
x8 = x(:,8);

%Define inputs
u1 = u(:,1);
u2 = u(:,2);

%Define ODE right-hand side
dx(:,1) = x4;
dx(:,2) = x5;
dx(:,3) = x6;

% Elevation Friction Model with Actuator Lag
dx(:,4) = ((-0.4e1 .* (cos(x2) .^ 2 - 0.2e1) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .* (mb - mf) .* La .* sin(x2) .* x6 .^ 2 .* cos(x1) .^ 3 + (0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* Lh .* (mb - mf) .* La .* x6 .* cos(x2) .^ 3 + (-0.8e1 .* (mf + mb) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .^ 2 .* x6 .^ 2 .* sin(x1) + ((-0.8e1 .* x5 .* x6 .* Lh .^ 2 .* mf + Lw .* mw .* grav) .* mb .^ 2 + (-0.8e1 .* Lh .^ 2 .* mf .^ 2 .* x5 .* x6 - 0.2e1 .* Lw .* grav .* mf .* mw) .* mb + grav .* Lw .* mf .^ 2 .* mw) .* La .^ 2 - 0.4e1 .* ((Lh .^ 2 .* mf - Lw .^ 2 .* mw ./ 0.4e1) .* mb .^ 2 + mf .* (Lh .^ 2 .* mf + Lw .^ 2 .* mw ./ 0.2e1) .* mb - Lw .^ 2 .* mf .^ 2 .* mw ./ 0.4e1) .* grav .* La + Lh .^ 2 .* Lw .* mw .* (mf + mb) .^ 2 .* (-0.2e1 .* Lw .* x5 .* x6 + grav)) .* cos(x2) .^ 2 - 0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* Lh .* (mb - mf) .* La .* x6 .* cos(x2) - 0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* (x6 .^ 2 .* ((mf + mb) .* La .^ 2 - Lh .^ 2 .* mb - Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* sin(x1) + La .* grav .* (mf + mb) - 0.2e1 .* x5 .* x6 .* Lh .^ 2 .* mb - 0.2e1 .* x5 .* x6 .* Lh .^ 2 .* mf - Lw .* mw .* grav)) .* cos(x1) .^ 2 + ((0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .^ 2 .* Lh .* (mb - mf) .* La .* sin(x2) + ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav + F_v1 .* x(4))) .* cos(x2) .^ 2 + (-0.8e1 .* (mf + mb) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* (x6 .* sin(x1) + x5) .* x(4) .* Lh .^ 2 .* sin(x2) - ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* (x8 - x7) .* (mb - mf) .* La .* Kf) .* cos(x2) - 0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* (Lh .* (mb - mf) .* ((0.2e1 .* x5 .* x6 .* La + grav) .* sin(x1) + La .* (x5 .^ 2 + x6 .^ 2)) .* sin(x2) + (-grav .* mb - grav .* mf - Kf .* (x8 + x7)) .* La + Lw .* mw .* grav - F_v1 .* x(4))) .* cos(x1) + ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .* (((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav) .* cos(x2) .^ 2 - Kf .* sin(x1) .* Lh .* (x8 - x7) .* sin(x2) + (-grav .* mb - grav .* mf - Kf .* (x8 + x7)) .* La + Lw .* mw .* grav)) .* cosh(0.10e3 .* x1) + F_c1 .* cos(x1) .* sinh(0.10e3 .* x1) .* (((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 + 0.4e1 .* La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb))) ./ (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) ./ cosh(0.10e3 .* x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) ./ cos(x1) ./ 0.4e1;
dx(:,5) = ((-0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .* sin(x2) .* x6 .^ 2 .* ((-mb - mf) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* cos(x2) .* cos(x1) .^ 4 + (0.4e1 .* sin(x1) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .^ 2 .* (mb - mf) .* La .* x6 .^ 2 .* cos(x2) .^ 2 + 0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x(4) .* Lh .* x6 .* cos(x2) + (mb - mf) .* (-0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .^ 2 .* La .* x6 .^ 2 .* sin(x1) - 0.4e1 .* (0.2e1 .* x5 .* x6 .* La .^ 2 .* mb .* mf + grav .* La .* mb .* mf - Lw .* mw .* (mf + mb) .* (-0.2e1 .* Lw .* x5 .* x6 + grav) ./ 0.4e1) .* La .* Lh .^ 2 + ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* Lw .* (La + Lw) .* grav .* mw)) .* cos(x2) .* cos(x1) .^ 3 + (-Lh .^ 2 .* (-0.8e1 .* sin(x1) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* (mb - mf) .* La .* x6 .* sin(x2) + ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* (x8 - x7) .* Kf) .* cos(x2) .^ 2 + (((-0.4e1 .* (mf + mb) .* (0.2e1 .* x5 .* x6 .* La .^ 2 .* mb .* mf + grav .* La .* mb .* mf - Lw .* mw .* (mf + mb) .* (-0.2e1 .* Lw .* x5 .* x6 + grav) ./ 0.4e1) .* Lh .^ 2 + grav .* La .* Lw .* mw .* (mb - mf) .^ 2 .* (La + Lw)) .* sin(x1) + 0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* ((x(4) .^ 2 - 0.2e1 .* x6 .^ 2) .* (mf + mb) .* Lh .^ 2 + ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x(4) .^ 2)) .* Lh .* sin(x2) + (Lh .^ 2 .* (mf + mb) + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (mb - mf) .* La .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav + F_v1 .* x(4))) .* cos(x2) - (Lh .^ 2 .* (mf + mb) + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (x8 - x7) .* ((-mb - mf) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* Kf) .* cos(x1) .^ 2 + (-0.4e1 .* sin(x1) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .^ 2 .* Lh .^ 2 .* (mb - mf) .* La .* cos(x2) .^ 3 + (0.8e1 .* (mf + mb) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* Lh .^ 3 .* x5 .* sin(x1) + 0.8e1 .* (mf + mb) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* x6 .* Lh .^ 3 + (mf + mb) .* (mb - mf) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav) .* La .* Lh .^ 2 + ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (mb - mf) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav) .* La) .* cos(x2) .^ 2 + Lh .* (sin(x1) .* ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav + F_v1 .* x(4)) .* sin(x2) + 0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .* (mb - mf) .* (La .* (x5 .^ 2 + x6 .^ 2) .* sin(x1) + 0.2e1 .* x5 .* x6 .* La + grav)) .* cos(x2) - (Lh .^ 2 .* (mf + mb) + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (0.2e1 .* Kf .* sin(x1) .* La .* Lh .* (mb - mf) .* (x8 - x7) .* sin(x2) + 0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* x6 .* Lh + (mb - mf) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav) .* La)) .* cos(x1) + Lh .* (sin(x1) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav) .* sin(x2) + Kf .* Lh .* (x8 - x7)) .* (((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 - (mf + mb) .* (Lh .^ 2 .* (mf + mb) + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw))) .* cosh(0.10e3 .* x1) + F_c1 .* ((Lh .^ 2 .* (mf + mb) + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (mb - mf) .* La .* cos(x1) + sin(x1) .* ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* Lh .* sin(x2)) .* cos(x1) .* sinh(0.10e3 .* x1) .* cos(x2)) ./ (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) ./ cosh(0.10e3 .* x1) ./ Lh ./ (Lh .^ 2 .* (mf + mb) + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) ./ cos(x1) .^ 2 ./ 0.4e1;
dx(:,6) = ((0.4e1 .* (x6 .* cos(x1) + x(4)) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .* cos(x1) .* (mb - mf) .* La .* (-x6 .* cos(x1) + x(4)) .* cos(x2) .^ 3 + ((-0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* Lh .* (mb - mf) .* La .* x6 .* cos(x1) .^ 2 - ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav)) .* sin(x2) - Lh .* (0.8e1 .* (mf + mb) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* (x6 .* sin(x1) + x5) .* x(4) .* Lh .* cos(x1) + sin(x1) .* ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* (x8 - x7) .* Kf)) .* cos(x2) .^ 2 - cos(x1) .* (((-0.8e1 .* (mf + mb) .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* Lh .^ 2 .* x6 .^ 2 .* sin(x1) + ((-0.8e1 .* x5 .* x6 .* Lh .^ 2 .* mf + Lw .* mw .* grav) .* mb .^ 2 + (-0.8e1 .* Lh .^ 2 .* mf .^ 2 .* x5 .* x6 - 0.2e1 .* Lw .* grav .* mf .* mw) .* mb + grav .* Lw .* mf .^ 2 .* mw) .* La .^ 2 - 0.4e1 .* ((Lh .^ 2 .* mf - Lw .^ 2 .* mw ./ 0.4e1) .* mb .^ 2 + mf .* (Lh .^ 2 .* mf + Lw .^ 2 .* mw ./ 0.2e1) .* mb - Lw .^ 2 .* mf .^ 2 .* mw ./ 0.4e1) .* grav .* La + Lh .^ 2 .* Lw .* mw .* (mf + mb) .^ 2 .* (-0.2e1 .* Lw .* x5 .* x6 + grav)) .* cos(x1) + ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav + F_v1 .* x(4))) .* sin(x2) + 0.4e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* (-0.2e1 .* x6 .^ 2 .* La .* cos(x1) .^ 2 + (0.2e1 .* x5 .* x6 .* La + grav) .* sin(x1) + La .* (x5 .^ 2 + x6 .^ 2)) .* Lh .* (mb - mf)) .* cos(x2) + ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* ((Kf .* La .* (mb - mf) .* (x8 - x7) .* cos(x1) + (mf + mb) .* ((grav .* mb + grav .* mf + Kf .* (x8 + x7)) .* La - Lw .* mw .* grav)) .* sin(x2) + sin(x1) .* (0.8e1 .* (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) .* x(4) .* x6 .* cos(x1) + Kf .* Lh .* (mf + mb) .* (x8 - x7)))) .* cosh(0.10e3 .* x1) - ((mb - mf) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* F_c1 .* cos(x1) .* sin(x2) .* sinh(0.10e3 .* x1) .* cos(x2)) ./ (La .^ 2 .* mb .* mf + Lw .^ 2 .* mw .* (mf + mb) ./ 0.4e1) ./ cosh(0.10e3 .* x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) ./ cos(x1) .^ 2 ./ 0.4e1;
dx(:,7) = ta*x7 - ta*u1;
dx(:,8) = ta*x8 - ta*u2;

% % Nonlinear Frictionless Actuator Lag model
% dx(:,4) = -cos(x2) .^ 2 .* Lh .* (mf - mb) .* La .* sin(x2) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x4 .^ 2 + (-0.2e1 .* Lh .* cos(x2) .* (cos(x1) .* La .* (mf - mb) .* cos(x2) .^ 2 - cos(x1) .* La .* (mf - mb) + sin(x2) .* sin(x1) .* Lh .* (mf + mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 - 0.2e1 .* (mf + mb) .* cos(x2) .* Lh .^ 2 .* sin(x2) .* x5 ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) .* x4 + ((sin(x2) .* La .* (mf - mb) .* cos(x1) - 0.2e1 .* sin(x1) .* Lh .* (mf + mb)) .* Lh .* cos(x1) .* cos(x2) .^ 2 - 0.2e1 .* sin(x2) .* La .* Lh .* (mf - mb) .* cos(x1) .^ 2 - sin(x1) .* ((mf + mb) .* La .^ 2 - Lh .^ 2 .* mb - Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* cos(x1) + sin(x2) .* La .* Lh .* (mf - mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 .^ 2 - 0.2e1 .* Lh .* (cos(x1) .* Lh .* (mf + mb) .* cos(x2) .^ 2 - cos(x1) .* Lh .* (mf + mb) - sin(x2) .* sin(x1) .* La .* (mf - mb)) .* x5 ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 + Lh .* (mf - mb) .* La .* sin(x2) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x5 .^ 2 + (((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 3 + (grav .* (mw .* Lw .* (mf - mb) .^ 2 .* La .^ 2 + ((-0.4e1 .* Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* mb .^ 2 + (-0.4e1 .* Lh .^ 2 .* mf .^ 2 - 0.2e1 .* Lw .^ 2 .* mw .* mf) .* mb + mf .^ 2 .* mw .* Lw .^ 2) .* La + mw .* Lh .^ 2 .* Lw .* (mf + mb) .^ 2) .* cos(x1) + ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2)) .* cos(x1) .* cos(x2) .^ 2 + Kf .* ((x8 - x7) .* (mf - mb) .* ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* La .* cos(x1) - (sin(x1) .* Lh .* Kf .* (-x7 + x8) .* sin(x2) + (grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2)) .* cos(x2) + (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) .* cos(x1) .* (-grav .* ((mf + mb) .* La - Lw .* mw) .* cos(x1) + grav .* sin(x1) .* Lh .* (mf - mb) .* sin(x2) + (grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav)) ./ (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) ./ cos(x1);
% dx(:,5) = cos(x2) .* (((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* sin(x2) .* cos(x1) + cos(x2) .^ 2 .* sin(x1) .* La .* Lh .* (mf - mb)) ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x4 .^ 2 + (-0.2e1 .* (-cos(x2) .^ 2 .* ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* cos(x1) .^ 2 + cos(x2) .^ 2 .* sin(x2) .* sin(x1) .* La .* Lh .* (mf - mb) .* cos(x1) - Lh .^ 2 .* (mf + mb) .* cos(x2) .^ 2 + (mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x6 + 0.2e1 .* (mf + mb) .* cos(x2) .^ 2 .* Lh .^ 2 .* sin(x1) .* x5 ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw)) .* x4 - cos(x2) .* (((-mb - mf) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* sin(x2) .* cos(x1) .^ 3 + sin(x1) .* La .* Lh .* (cos(x2) .^ 2 - 0.2e1) .* (mf - mb) .* cos(x1) .^ 2 + 0.2e1 .* sin(x2) .* Lh .^ 2 .* (mf + mb) .* cos(x1) + sin(x1) .* La .* Lh .* (mf - mb)) ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x6 .^ 2 - 0.2e1 .* Lh .* cos(x2) .* (-La .* (mf - mb) .* cos(x1) .^ 2 + sin(x2) .* sin(x1) .* Lh .* (mf + mb) .* cos(x1) + La .* (mf - mb)) .* x5 ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x6 - cos(x2) .* Lh .* sin(x1) .* (mf - mb) .* La ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x5 .^ 2 + (-grav .* cos(x2) .* (mf - mb) .* ((-0.4e1 .* La .* mb .* mf + mw .* Lw .* (mf + mb)) .* La .* Lh .^ 2 + Lw .* ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (La + Lw) .* mw) .* cos(x1) .^ 3 + (-Lh .^ 2 .* Kf .* (-x7 + x8) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 + (grav .* Lh .* sin(x1) .* ((mf + mb) .* (-0.4e1 .* La .* mb .* mf + mw .* Lw .* (mf + mb)) .* Lh .^ 2 + Lw .* La .* mw .* (mf - mb) .^ 2 .* (La + Lw)) .* sin(x2) - ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* (mf - mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* La) .* cos(x2) - Kf .* (-x7 + x8) .* ((-mb - mf) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw)) .* cos(x1) .^ 2 + (-((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* (mf - mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* La .* cos(x2) .^ 2 + Lh .* (((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* sin(x1) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* sin(x2) - grav .* Lh .* (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) .* (mf - mb)) .* cos(x2) + (0.2e1 .* sin(x1) .* Lh .* Kf .* (-x7 + x8) .* sin(x2) + (grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* (mf - mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* La) .* cos(x1) + Lh .* (((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* sin(x1) .* sin(x2) + Lh .* Kf .* (-x7 + x8)) .* (((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 - (mf + mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw))) ./ Lh ./ (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) ./ cos(x1) .^ 2;
% dx(:,6) = -cos(x2) .^ 3 .* Lh .* (mf - mb) .* La ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x4 .^ 2 + (0.2e1 .* (Lh .* (sin(x2) .* La .* (mf - mb) .* cos(x1) - sin(x1) .* Lh .* (mf + mb)) .* cos(x2) .^ 2 + sin(x1) .* ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 - 0.2e1 .* (mf + mb) .* cos(x2) .^ 2 .* Lh .^ 2 .* x5 ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) .* x4 + cos(x2) .* Lh .* (La .* (mf - mb) .* cos(x1) .^ 2 .* cos(x2) .^ 2 + 0.2e1 .* sin(x2) .* sin(x1) .* Lh .* (mf + mb) .* cos(x1) - 0.2e1 .* (mf - mb) .* (cos(x1) .^ 2 - 0.1e1 ./ 0.2e1) .* La) ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 .^ 2 + 0.2e1 .* (cos(x1) .* Lh .* (mf + mb) .* sin(x2) + sin(x1) .* La .* (mf - mb)) .* cos(x2) .* Lh .* x5 ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 + cos(x2) .* Lh .* (mf - mb) .* La ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x5 .^ 2 + (-(((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* sin(x2) + sin(x1) .* Lh .* Kf .* (-x7 + x8)) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 - ((grav .* (mw .* Lw .* (mf - mb) .^ 2 .* La .^ 2 + ((-0.4e1 .* Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* mb .^ 2 + (-0.4e1 .* Lh .^ 2 .* mf .^ 2 - 0.2e1 .* Lw .^ 2 .* mw .* mf) .* mb + mf .^ 2 .* mw .* Lw .^ 2) .* La + mw .* Lh .^ 2 .* Lw .* (mf + mb) .^ 2) .* cos(x1) + ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2)) .* sin(x2) - grav .* Lh .* sin(x1) .* (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) .* (mf - mb)) .* cos(x1) .* cos(x2) + ((- La .* Kf .* (-x7 + x8) .* (mf - mb) .* cos(x1) + (mf + mb) .* ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav)) .* sin(x2) + sin(x1) .* Lh .* Kf .* (-x7 + x8) .* (mf + mb)) .* ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) ./ (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) ./ cos(x1) .^ 2;
% dx(:,7) = ta*x7 - ta*u1;
% dx(:,8) = ta*x8 - ta*u2;



%------------- END OF CODE --------------


function c = g(x,u,p,t,data)

% g - Returns the path constraint function where gl =< g(x,u,p,t) =< gu
% The function must be vectorised and
% xi, ui, pi are column vectors taken as x(:,i), u(:,i) and p(:,i). Each
% constraint corresponds to one column of c
% 
% Syntax:  c=g(x,u,p,t,data)
%
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%   data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    c - constraint function
%
%------------- BEGIN CODE --------------

c = [];

%------------- END OF CODE --------------

function bc = b(x0,xf,u0,uf,p,tf,data)

% b - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b(x0,xf,u0,uf,p,tf,data)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------

bc = [];

%------------- END OF CODE --------------






