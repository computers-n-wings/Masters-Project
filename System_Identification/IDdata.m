function [se, sv] = IDdata
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Data

    finish = 6004;
    start = 4;

    A = 'elevation';
    B = 'voltage';
    C = 'pitch';
    D = 'travel';
    for i = 1:3
        Y = [];
        U = [];
        z = [];
        % Elevation
        filename = strcat(A, int2str(i), '.txt');
        Y = [Y pi * importfile3(filename, start+1, finish) / 180];
%         % Pitch
        filename = strcat(C, int2str(i), '.txt');
        Y = [Y pi * importfile3(filename, start+1, finish) / 180];
        % Travel
        filename = strcat(D, int2str(i), '.txt');
        Y = [Y pi * importfile3(filename, start+1, finish) / 180];
        % Elevation Rate
        Y = [Y smooth(gradient(Y(:,1),0.005),0.1, 'loess')];
        % Pitch Rate
        Y = [Y smooth(gradient(Y(:,2),0.005),0.1, 'loess')];
        % Travel Rate
        Y = [Y smooth(gradient(Y(:,3),0.005),0.1, 'loess')];
        % Voltage
        filename = strcat(B, int2str(i), '.txt');
        [U(:,1),U(:,2)] = importfile2(filename, start+1, finish);
        % ID Data
        z = iddata(Y, U, 5e-3);
        if i == 3
            sv = z;
        else
            se{i} = z;
        end
    end
end