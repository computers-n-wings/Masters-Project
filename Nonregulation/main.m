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

[problem, guess] = Nonregulation;
options = settings;
plant = @testPlant;

[infoNLP,data] = transcribeOCP(problem, guess, options);
[nt,np,n,m,ng,nb,M,N,ns]=deal(data.sizes{:});
time=[];states=[]; inputs=[];status_mpc=[];comp_time = 0;sol_time=[];

x_des = [0;0;pi;0;0;0];
tc = 0.05;
P=5/tc;

for i = 1:P
    
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

    time=[time;tv];                                 % Store results
    status_mpc=[status_mpc;status]; 
    states=[states;xv]; 
    inputs=[inputs;uv];
    sol_time=[sol_time;solution.computation_time];
    comp_time = comp_time + solution.computation_time;

    infoNLP.zl(nt+np+1:nt+np+n)=x0;                 % Update initial condition  
    infoNLP.zu(nt+np+1:nt+np+n)=x0;  

    infoNLP.z0=solution.z;                          % Update initial guess
    data.x0t=x0';
    if norm(data.x0t(1:6)-x_des, inf) < 0.1
        break
    end

end

figure(1)
plot(time, (180/pi)*states(:,1), 'LineWidth',1.5);
hold on;
plot(Trajectory.T, (180/pi)*Trajectory.X(:, 1), 'LineWidth',1.5);
legend({'Generated', 'Optimal'}, 'Location', 'southwest', 'FontSize', 11);
title('Non Regulation Control: Elevation', 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 14);
ylabel('Elevation (degrees)', 'FontSize', 14);
grid on;

figure(2)
plot(time, (180/pi)*states(:,2), 'LineWidth',1.5);
hold on;
plot(Trajectory.T, (180/pi)*Trajectory.X(:, 2), 'LineWidth',1.5);
hold on;
plot(time, 90*ones(size(time)), '--k');
hold on;
plot(time, -90*ones(size(time)), '--k');
legend({'Generated', 'Optimal', 'Constraint'}, 'Location', 'northwest', 'FontSize', 11);
title('Non Regulation Control: Pitch', 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 14);
ylabel('Pitch (degrees)', 'FontSize', 14);
grid on;

figure(3)
plot(time, (180/pi)*states(:,3), 'LineWidth',1.5);
hold on;
plot(Trajectory.T, (180/pi)*Trajectory.X(:, 3), 'LineWidth',1.5);
legend({'Generated', 'Optimal'}, 'Location', 'northwest', 'FontSize', 11);
title('Non Regulation Control: Travel', 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 14);
ylabel('Travel (degrees)', 'FontSize', 14);
grid on;
    