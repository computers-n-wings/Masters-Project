clear all

addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/


begin = 102;
finish = 402;
G = [];

figure();
t = linspace(1, 2.5, finish - begin)';
for i = 1:3
    filename = strcat('Max_Pitch_Rate_', int2str(i), '.txt');
    P = importfile(filename, begin+1, finish);
%     G = [G, polyfit(t, P, 1)];
    plot(t, P);
    hold on
end