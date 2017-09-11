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

%% Make sure to link these to your libraries!!
addpath /home/jcg216//Work/ICLOCS_1.2.1/src
addpath /home/jcg216/CoinIpopt/build/lib
filename = 'NLFAL3.mat';

[problem,guess] = Global_Ref_Trajectory;                % Fetch the problem definition
options = settings;                                     % Get options and solver settings         

[infoNLP,data] = transcribeOCP(problem,guess,options);  % Format for NLP solver

[solution,status] = solveNLP(infoNLP,data);             % Solve the NLP

figure(1)
 plot(solution.T,solution.X(:,1)*(180/pi), 'b', 'LineWidth',1.5);
 hold on
 plot(solution.T, solution.X(:,2)*(180/pi),'r', 'LineWidth',1.5);
 hold on
 plot(solution.T, solution.X(:,3)*(180/pi), 'k', 'LineWidth',1.5);
 hold on
 plot(solution.T, 90*ones(size(solution.T)), '--r', 'LineWidth',1 );
 hold on;
 plot(solution.T, 27.5*ones(size(solution.T)), '--b', 'LineWidth',1 );
 hold on;
 plot(solution.T, -27.5*ones(size(solution.T)), '--b', 'LineWidth',1 );
 hold on;
 plot(solution.T, -90*ones(size(solution.T)), '--r', 'LineWidth',1 );
 title('Optimal State Trajectory', 'FontSize', 16)
 xlabel('Time (s)', 'FontSize', 14)
 ylabel('Position (degrees)', 'FontSize', 14)
 legend({'Elevation', 'Pitch', 'Travel', 'Pitch Constraint', 'Elevation Constraint'}, 'Location', 'northwest', 'FontSize', 11);
 grid on;
 
 figure(2)
 stairs(solution.T,solution.U(:,1), 'b', 'LineWidth',1.5);
 hold on
 stairs(solution.T, solution.U(:,2),'r', 'LineWidth',1.5);
 hold on
 plot(solution.T, 24*ones(size(solution.T)), '--k', 'LineWidth',1 );
 hold on
 plot(solution.T, -24*ones(size(solution.T)), '--k', 'LineWidth',1 );
 title('Inputs for Optimal Trajectory', 'FontSize', 16)
 xlabel('Time (s)', 'FontSize', 14)
 ylabel('Voltage (V)', 'FontSize', 14)
 legend({'Front Motor', 'Back Motor', 'Input Constraint'}, 'Location', 'north', 'FontSize', 11);
 grid on;
 
 Trajectory = solution;
 save(filename, 'Trajectory');
 

