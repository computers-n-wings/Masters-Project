function [U1, U2] = importdata1()
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Data

    filename = strcat('voltage', int2str(1), '.txt');

    [time1,v1,time2,v2] = importvoltage(filename);

    U1 = [time1 v1];
    U2 = [time2 v2];
end