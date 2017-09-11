function [elev, pitch, trav] = lin_trajectory()
    clear all
    addpath /home/jcg216/Work/MSC_Project/Matlab_Files/Tracking_Reference/

    

    T = time;
    X = states';

    t = (0:0.005:3.4)';
    x = pchip(T,X,t)';

    elev = [t x(:,1)];
    pitch = [t x(:,2)];
    trav = [t x(:,3)];
end