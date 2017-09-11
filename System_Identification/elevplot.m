addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data1/
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data2/
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data3/

finish = 6104;
start = 104;

T = linspace(0, 30, finish-start);
Y = [];
A = 'Elev';
for k = 1
    V = k+4;
    for i = 1:3
        filename = strcat(int2str(i), '_', A, '_', int2str(V), '.txt');
        Y = [Y importfile(filename, start+1, finish)];
    end
end

figure(1)
for i = 1:3
    plot(T, Y(:,i));
    hold on;
end
 title('Elevation Response in Physical System (5V)')
 xlabel('Time (s)')
 ylabel('Elevation (degrees)')
 legend('Experiment 1', 'Experiment 2', 'Experiment 3', 'Location', 'southeast');
 grid on;