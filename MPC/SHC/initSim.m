    
clear all

addpath /home/jcg216/Work/ICLOCS_1.2.1/src
addpath /home/jcg216/CoinIpopt/build/lib

global infoNLP data sol_mpc t tstep
par=[];
sol_mpc=[];
tstep=0.005;
t = 1;

nodes=51;
options= settings_h(nodes);                  % Get options and solver settings
                         
options.ipopt.print_level=5;                   

[problem,guess]=Nonregulation;          % Fetch the problem definition

[infoNLP,data]=transcribeOCP(problem,guess,options); % Format for NLP solver

[nt,np,n,m,ng,nb,M,N,ns]=deal(data.sizes{1:9});
nz=length(infoNLP.z0);
nlambda=M*(n+ng)+nb;

