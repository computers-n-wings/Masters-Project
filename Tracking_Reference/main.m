% MAIN - Main script to solve the Optimal Control Problem
%
% Copyright (C) 2010 Paola Falugi, Eric Kerrigan and Eugene van Wyk. All Rights Reserved.
% This code is published under the BSD License.
% Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) 5 May 2010
% iclocs@imperial.ac.uk

%--------------------------------------------------------
clear all;
format compact;

addpath /home/jcg216/Work/ICLOCS_1.2.1/src
addpath /home/jcg216/CoinIpopt/build/lib
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/Global_Ref_Trajectory/
load NLFAL.mat

[problem, guess] = Tracking_Reference;
options = settings;
plant = @testPlant;

% TranscribeOCP has been altered to deal with time-varying reference
[infoNLP,data] = transcribeOCP(problem, guess, options);
[nt,np,n,m,ng,nb,M,N]=deal(data.sizes{1:8});
time=[];states=[]; inputs=[];status_mpc=[]; comp_time=0; Param=[];

% Extend the horizon of the reference
ref = Trajectory.X(:, 1:6);
tref = Trajectory.T;
for i = 1:500
    ref = [ref; Trajectory.X(end, 1:6)];
    tref = [tref; Trajectory.T(end) + i*Trajectory.tf/length(Trajectory.T)];
end

xr=pchip(tref,ref');

% t0 = Trajectory.T(1); tf = Trajectory.T(end); T = linspace(t0,tf,M);
t0 = 0; tf = 3; T = linspace(t0,tf,M);

tc = 0.01;
P = 3.5/tc;
x_des = [0;0;pi;0;0;0];

for i = 1:P
    
    data.references.xr = ppval(xr,T+(i-1)*tc)';

    disp('Compute Control Action. Iteration:');disp(i);
        
    [solution_new, status] = solveNLP(infoNLP,data);
    
    if status==0 || status==1 || i==1
        solution=solution_new;           
    else
        solution.x0=x0;
        solution.T=solution.T-tc;
    end

    disp('Apply Control')
    [x0,tv,xv,uv]=applyControl(tc,solution,plant,data,i); % Apply control 

    time=[time;tv(2:end)];                                 % Store results
    status_mpc=[status_mpc;status]; 
    states=[states;xv(2:end,:)]; 
    inputs=[inputs;uv(2:end,:)];
    comp_time=comp_time+solution.computation_time;
    
    infoNLP.zl(nt+np+1:nt+np+n)=x0;                 % Update initial condition  
    infoNLP.zu(nt+np+1:nt+np+n)=x0;  
    infoNLP.z0=solution.z;                          % Update initial guess
    data.x0t=x0';
    
    if norm(data.x0t(1:6)-x_des, inf)<0.1
        break
    end

end

plots(time,states,tref,ref)