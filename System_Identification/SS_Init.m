clear all
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/lib/
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/System_Identification/Max_Pitch/

[Kf, mh, mw, mf, mb, Lh, La, Lw, g] = setup_heli_3d_configuration();

s = IDdata_Pitch;

z = merge(s{2}, s{4}, s{5}, s{6}, s{7}, s{8}, s{9}, s{10}, s{11}, s{12});

c1 = -La/(2*(La^2)*mf+(Lw^2)*mw);

A = [0 c1 c1;0 -1 0; 0 0 -1];
B = [0 0; 0.12 0; 0 0.12];
C = [1 0 0];
D = [0 0];
K = zeros(3,1);
x0 = zeros(3,1);
Ts = 0;

init_sys = idss(A,B,C,D,K,x0,Ts);

init_sys.Structure.A.Free = [0 0 0; 0 1 0; 0 0 1];
init_sys.Structure.B.Free = [0 0; 1 0; 0 1];
init_sys.Structure.C.Free = false;
init_sys.Structure.D.Free = false;
init_sys.Structure.K.Free = false;

opt = ssestOptions;
opt.SearchOption.MaxIter = 200;
opt.Display = 'on';

sys = ssest(z, init_sys,opt)


