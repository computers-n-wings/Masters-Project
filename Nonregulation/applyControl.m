function [xf,tv,xv,uv]=applyControl(tc,solution,plant,data,k)
%APPLYCONTROL - Apply the control to the system
%
% Syntax:  xf = applyControl(tc,solution,plant,data,i)
%
% Inputs:
%    tc - Control horizon
%    solution - Structure with solution of NLP
%    data - Structure with usefull data
%    i - MPC iterate
%
% Outputs:
%    xf - State at time t=tc
%    tv - Time vector tv
%    xv - Plant states in time tv 
%    uv - Plant inputs applied at tv
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Copyright (C) 2010 Paola Falugi, Eric Kerrigan and Eugene van Wyk. All Rights Reserved.
% This code is published under the BSD License.
% Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) 5 May 2010
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------
[nt,np,n,m]=deal(data.sizes{1:4});

% Generate the control signal
U=solution.U; Up=cell(1,m);U(end,:)=[];
t0=tc*(k-1);
tf=t0+tc;
T=solution.T+t0;
p=solution.p;
xr=pchip(T,solution.X');
K=0;      % Option not active: Intersample feedback  (solution.K=0); 


for i=1:m % Piecewise constant polynomials
    Up{i}=mkpp(T,U(:,i)');
end

% Integrate equations with Matlab ODE solvers
[tv xv]=ode113(@(t,x)odewrap(t,x,Up,p,plant,K,xr,data),[t0 tf],solution.x0);
uv=cell2mat(cellfun(@(Up)ppval(Up,tv),Up,'UniformOutput',false));
uv(end,:)=uv((end-1),:);
xf=xv(end,:);
tv(end)=[];
xv(end,:)=[];


function dx=odewrap(t,x,Up,p,f,K,xr,data)
% Define inputs at necessary values
u=cellfun(@(Up)ppval(Up,t),Up);

dx=f(x',u,p,t,data)';


%------------- END OF CODE --------------
