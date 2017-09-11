function s = IDdata_Elev()
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data1/
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data2/
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data3/

    finish = 3204;
    start = 204;

    A = 'Elev';
    for k = 1:5
        V = k+4;
        for i = 1:3
            filename = strcat(int2str(i), '_', A, '_', int2str(V), '.txt');
            Y = pi * importfile(filename, start+1, finish) / 180;
            U = V*[ones(finish-start, 2)];
            s{(k-1)*3+i} = iddata(Y, U, 5e-3);
        end
    end
end