function u = Calculate_input(time_status)

    global data sol_mpc tstep
    
    time = time_status(1);
    status = time_status(2);
    
    [nt,np,n,m,ng,nb,M,N,ns]=deal(data.sizes{1:9});

    U1 = sol_mpc.U;Up=cell(1,m);
    T = sol_mpc.T+time;
    T1 = (tstep:tstep:N*tstep)'+time;
    
    Up=cell(1,m);U1(end,:)=[];
    
    for i=1:m % Piecewise constant polynomials
        Up{i}=mkpp(T,U1(:,i)');
    end
    
    u=cell2mat(cellfun(@(Up)ppval(Up,T1),Up,'UniformOutput',false));
end