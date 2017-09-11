
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Data
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/Simulink/
load('elev_friction.mat');
load('elevation_fl.mat');

finish = 4004;
start = 4;
T = linspace(0,20, finish-start)';

A = 'elevation';
filename = strcat(A, int2str(1), 'a', '.txt');
Y = importfile1(filename, start+1, finish);

plot(tout, elevation_ref, 'k', 'LineWidth',1.5);
hold on
plot(T,Y, 'r', 'LineWidth',1.5);
hold on
plot(tout, elevation_fl, '--b', 'LineWidth',1.5);
hold on
plot(tout, elevation_friction, ':b', 'LineWidth',1.5);
title('Closed-loop Responses', 'FontSize', 16)
xlabel('Time (s)', 'FontSize', 14)
ylabel('Elevation (degrees)', 'FontSize', 14)
legend({'Reference', 'Physical System', 'Frictionless Model', 'Friction Model'}, 'Location', 'southeast', 'FontSize', 11);
grid on;