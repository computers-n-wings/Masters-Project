function [ zplambda ] = ICLOCS_NLPSolver(x0pz0)

global infoNLP data t sol_mpc xr tstep T iter

[nt,np,n,m,ng,nb,M,N,ns]=deal(data.sizes{1:9});
nz=length(infoNLP.z0);

x0=x0pz0(1:n);
z0=x0pz0(n+1:n+nz);
lambda0=x0pz0(n+nz+1:end);
infoNLP.zl(nt+np+1:nt+np+n)=x0;                 % Update initial condition  
infoNLP.zu(nt+np+1:nt+np+n)=x0;  
infoNLP.z0=z0;   % Update initial guess
data.x0t=x0; 
data.x0=x0; 
if any(lambda0)
    data.multipliers.lambda=lambda0;
end

data.references.xr = ppval(xr,T + tstep*iter)';

[sol_mpc,status] = solveNLP(infoNLP,data);      % Solve the NLP

if status == 0 || status == 1
    t = 1;
    zplambda=[sol_mpc.z;sol_mpc.multipliers.lambda;status];
else
    t = t+1;
    zplambda=[z0;lambda0;status];
end

iter = iter+1;

end

