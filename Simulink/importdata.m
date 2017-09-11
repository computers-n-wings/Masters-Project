function [Elev, Pitch, Trav] = importdata()
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Data

    A = 'elevation';
    C = 'pitch';
    D = 'travel';

    % Elevation
    filename = strcat(A, int2str(1), 'a', '.txt');
    [time,elev] = importelev(filename);
    Elev = [time pi*elev/180];
    % Pitch
    filename = strcat(C, int2str(1), 'a', '.txt');
    [time, pitch] = importpitch(filename);
    Pitch = [time pi*pitch/180];
    % Travel
    filename = strcat(D, int2str(1), 'a', '.txt');
    [time, trav] = importtrav(filename);
    Trav = [time pi*trav/180];
end