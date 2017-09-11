function s = IDdata_Pitch()
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data_Pitch_1/
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data_Pitch_2/
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Heli_Data_Pitch_3/

    finish = 400;
    begin = 230;

    u0 = zeros(finish-begin, 1);
    u1 = ones(finish-begin, 1);
    U = [];
    A = 'Pitch';
    for k = 1:5
        for i = 1:3
            filename = strcat(int2str(i), '_', A, '_', int2str(k+3), int2str(0), '.txt');
            Y = importfile(filename, begin+1, finish)*(pi/180);
            for j = 1:length(Y)
                Y(j) = Y(j) - Y(1);
            end
            Y = [zeros(30,1); Y];
            Y = gradient(Y, 5e-3);
            Y = smooth(Y,0.1, 'loess');
            U = [zeros(30,2); u0, (k+3)*u1];
            s{(k-1)*3+i} = iddata(Y, U, 5e-3);
        end
    end
end