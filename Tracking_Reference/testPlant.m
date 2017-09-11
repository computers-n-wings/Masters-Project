function dx = testPlant(x,u,p,t,data)

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

Lh = data.data.Lh; mf = data.data.mf; mb = data.data.mb; mw = data.data.mw;
Kf = data.data.Kf; grav = data.data.grav; La = data.data.La; Lw = data.data.Lw;
F_v1 = data.data.F_v1; 
F_c1 = data.data.F_c1; 
ta = data.data.ta;

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

% % Frictionless Model with Actuator Lag
% dx(:,4) = -cos(x2) .^ 2 .* Lh .* (mf - mb) .* La .* sin(x2) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x4 .^ 2 + (-0.2e1 .* Lh .* cos(x2) .* (cos(x1) .* La .* (mf - mb) .* cos(x2) .^ 2 - cos(x1) .* La .* (mf - mb) + sin(x2) .* sin(x1) .* Lh .* (mf + mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 - 0.2e1 .* (mf + mb) .* cos(x2) .* Lh .^ 2 .* sin(x2) .* x5 ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) .* x4 + ((sin(x2) .* La .* (mf - mb) .* cos(x1) - 0.2e1 .* sin(x1) .* Lh .* (mf + mb)) .* Lh .* cos(x1) .* cos(x2) .^ 2 - 0.2e1 .* sin(x2) .* La .* Lh .* (mf - mb) .* cos(x1) .^ 2 - sin(x1) .* ((mf + mb) .* La .^ 2 - Lh .^ 2 .* mb - Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* cos(x1) + sin(x2) .* La .* Lh .* (mf - mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 .^ 2 - 0.2e1 .* Lh .* (cos(x1) .* Lh .* (mf + mb) .* cos(x2) .^ 2 - cos(x1) .* Lh .* (mf + mb) - sin(x2) .* sin(x1) .* La .* (mf - mb)) .* x5 ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 + Lh .* (mf - mb) .* La .* sin(x2) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x5 .^ 2 + (((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 3 + (grav .* (mw .* Lw .* (mf - mb) .^ 2 .* La .^ 2 + ((-0.4e1 .* Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* mb .^ 2 + (-0.4e1 .* Lh .^ 2 .* mf .^ 2 - 0.2e1 .* Lw .^ 2 .* mw .* mf) .* mb + mf .^ 2 .* mw .* Lw .^ 2) .* La + mw .* Lh .^ 2 .* Lw .* (mf + mb) .^ 2) .* cos(x1) + ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2)) .* cos(x1) .* cos(x2) .^ 2 + Kf .* ((x8 - x7) .* (mf - mb) .* ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* La .* cos(x1) - (sin(x1) .* Lh .* Kf .* (-x7 + x8) .* sin(x2) + (grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2)) .* cos(x2) + (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) .* cos(x1) .* (-grav .* ((mf + mb) .* La - Lw .* mw) .* cos(x1) + grav .* sin(x1) .* Lh .* (mf - mb) .* sin(x2) + (grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav)) ./ (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) ./ cos(x1);
% dx(:,5) = cos(x2) .* (((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* sin(x2) .* cos(x1) + cos(x2) .^ 2 .* sin(x1) .* La .* Lh .* (mf - mb)) ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x4 .^ 2 + (-0.2e1 .* (-cos(x2) .^ 2 .* ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* cos(x1) .^ 2 + cos(x2) .^ 2 .* sin(x2) .* sin(x1) .* La .* Lh .* (mf - mb) .* cos(x1) - Lh .^ 2 .* (mf + mb) .* cos(x2) .^ 2 + (mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x6 + 0.2e1 .* (mf + mb) .* cos(x2) .^ 2 .* Lh .^ 2 .* sin(x1) .* x5 ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw)) .* x4 - cos(x2) .* (((-mb - mf) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* sin(x2) .* cos(x1) .^ 3 + sin(x1) .* La .* Lh .* (cos(x2) .^ 2 - 0.2e1) .* (mf - mb) .* cos(x1) .^ 2 + 0.2e1 .* sin(x2) .* Lh .^ 2 .* (mf + mb) .* cos(x1) + sin(x1) .* La .* Lh .* (mf - mb)) ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x6 .^ 2 - 0.2e1 .* Lh .* cos(x2) .* (-La .* (mf - mb) .* cos(x1) .^ 2 + sin(x2) .* sin(x1) .* Lh .* (mf + mb) .* cos(x1) + La .* (mf - mb)) .* x5 ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x6 - cos(x2) .* Lh .* sin(x1) .* (mf - mb) .* La ./ cos(x1) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* x5 .^ 2 + (-grav .* cos(x2) .* (mf - mb) .* ((-0.4e1 .* La .* mb .* mf + mw .* Lw .* (mf + mb)) .* La .* Lh .^ 2 + Lw .* ((mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* (La + Lw) .* mw) .* cos(x1) .^ 3 + (-Lh .^ 2 .* Kf .* (-x7 + x8) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 + (grav .* Lh .* sin(x1) .* ((mf + mb) .* (-0.4e1 .* La .* mb .* mf + mw .* Lw .* (mf + mb)) .* Lh .^ 2 + Lw .* La .* mw .* (mf - mb) .^ 2 .* (La + Lw)) .* sin(x2) - ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* (mf - mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* La) .* cos(x2) - Kf .* (-x7 + x8) .* ((-mb - mf) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw)) .* cos(x1) .^ 2 + (-((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* (mf - mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* La .* cos(x2) .^ 2 + Lh .* (((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* sin(x1) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* sin(x2) - grav .* Lh .* (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) .* (mf - mb)) .* cos(x2) + (0.2e1 .* sin(x1) .* Lh .* Kf .* (-x7 + x8) .* sin(x2) + (grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* (mf - mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) .* La) .* cos(x1) + Lh .* (((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* sin(x1) .* sin(x2) + Lh .* Kf .* (-x7 + x8)) .* (((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 - (mf + mb) .* ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw))) ./ Lh ./ (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) ./ ((mf + mb) .* Lh .^ 2 + (mf + mb) .* La .^ 2 + Lw .^ 2 .* mw) ./ cos(x1) .^ 2;
% dx(:,6) = -cos(x2) .^ 3 .* Lh .* (mf - mb) .* La ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x4 .^ 2 + (0.2e1 .* (Lh .* (sin(x2) .* La .* (mf - mb) .* cos(x1) - sin(x1) .* Lh .* (mf + mb)) .* cos(x2) .^ 2 + sin(x1) .* ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 - 0.2e1 .* (mf + mb) .* cos(x2) .^ 2 .* Lh .^ 2 .* x5 ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) .* x4 + cos(x2) .* Lh .* (La .* (mf - mb) .* cos(x1) .^ 2 .* cos(x2) .^ 2 + 0.2e1 .* sin(x2) .* sin(x1) .* Lh .* (mf + mb) .* cos(x1) - 0.2e1 .* (mf - mb) .* (cos(x1) .^ 2 - 0.1e1 ./ 0.2e1) .* La) ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 .^ 2 + 0.2e1 .* (cos(x1) .* Lh .* (mf + mb) .* sin(x2) + sin(x1) .* La .* (mf - mb)) .* cos(x2) .* Lh .* x5 ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x6 + cos(x2) .* Lh .* (mf - mb) .* La ./ cos(x1) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* x5 .^ 2 + (-(((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* sin(x2) + sin(x1) .* Lh .* Kf .* (-x7 + x8)) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2) .* cos(x2) .^ 2 - ((grav .* (mw .* Lw .* (mf - mb) .^ 2 .* La .^ 2 + ((-0.4e1 .* Lh .^ 2 .* mf + Lw .^ 2 .* mw) .* mb .^ 2 + (-0.4e1 .* Lh .^ 2 .* mf .^ 2 - 0.2e1 .* Lw .^ 2 .* mw .* mf) .* mb + mf .^ 2 .* mw .* Lw .^ 2) .* La + mw .* Lh .^ 2 .* Lw .* (mf + mb) .^ 2) .* cos(x1) + ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav) .* ((mf - mb) .^ 2 .* La .^ 2 + Lh .^ 2 .* (mf + mb) .^ 2)) .* sin(x2) - grav .* Lh .* sin(x1) .* (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) .* (mf - mb)) .* cos(x1) .* cos(x2) + ((- La .* Kf .* (-x7 + x8) .* (mf - mb) .* cos(x1) + (mf + mb) .* ((grav .* mb + grav .* mf + Kf .* (x7 + x8)) .* La - Lw .* mw .* grav)) .* sin(x2) + sin(x1) .* Lh .* Kf .* (-x7 + x8) .* (mf + mb)) .* ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw)) ./ (0.4e1 .* La .^ 2 .* mb .* mf + mw .* Lw .^ 2 .* (mf + mb)) ./ ((mf + mb) .* La .^ 2 + Lh .^ 2 .* mb + Lh .^ 2 .* mf + Lw .^ 2 .* mw) ./ cos(x1) .^ 2;
% dx(:,7) = ta*x7 - ta*u1;
% dx(:,8) = ta*x8 - ta*u2;




%------------- END OF CODE --------------