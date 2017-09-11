clear all

addpath /home/jcg216/Work/ICLOCS_1.2.1/src
addpath /home/jcg216/CoinIpopt/build/lib
addpath /home/jcg216/Work/MSC_Project/Matlab_Files/Global_Ref_Trajectory/
load NLFAL.mat

global infoNLP data sol_mpc t tstep xr T iter
par=[];
sol_mpc=[];
tstep=0.005;
t=1;
xr=[];
T=[];
iter=0;

nodes=51;
options= settings_h(nodes);                  % Get options and solver settings
                         
options.ipopt.print_level=5;                   

[problem,guess]=Tracking_Reference;          % Fetch the problem definition

[infoNLP,data]=transcribeOCP(problem,guess,options); % Format for NLP solver

[nt,np,n,m,ng,nb,M,N,ns]=deal(data.sizes{1:9});
nz=length(infoNLP.z0);
nlambda=M*(n+ng)+nb;

% Extend the horizon of the reference
ref = Trajectory.X(:, 1:6);
tref = Trajectory.T;
for i = 1:200
    ref = [ref; Trajectory.X(end, 1:6)];
    tref = [tref; Trajectory.T(end) + i*Trajectory.tf/M];
end

xr=pchip(tref,ref');

% t0 = Trajectory.T(1); tf = Trajectory.T(end); T = linspace(t0,tf,M);
t0 = 0; tf = 6;; T = linspace(t0,tf,M);