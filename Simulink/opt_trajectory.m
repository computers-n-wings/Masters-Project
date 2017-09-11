function [elev, pitch, trav] = opt_trajectory(text)
    
    filename = strcat(text, '.mat');
    load(filename);
    
    T = Trajectory.T;
    X = Trajectory.X';
    
    t = (0:0.005:2.75)';
    x = pchip(T,X,t)';

    elev = [t x(:,1)];
    pitch = [t x(:,2)];
    trav = [t x(:,3)];
end