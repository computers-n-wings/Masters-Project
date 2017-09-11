function s = IDdata_Elev1()
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Data

    finish = 6004;
    start = 54;

    A = 'elevation';
    B = 'voltage';
    C = 'pitch';
    D = 'travel';
    for i = 1:3
        Y= [];
        U= [];
        % Elevation
        filename = strcat(A, int2str(i), 'a', '.txt');
        Y = [Y pi * importfile1(filename, start+1, finish) / 180];
        % Voltage
        filename = strcat(B, int2str(i), 'a', '.txt');
        [U(:,1),U(:,2)] = importfile2(filename, start+1, finish);
        % ID Data
        s{i} = iddata(Y, U, 5e-3);
    end
end