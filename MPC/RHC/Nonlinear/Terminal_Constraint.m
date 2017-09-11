function Terminal_Constraint(s)

global infoNLP

if all(s~=(0&&1))
    infoNLP.zl(end-4:end-2) = -2*ones(3,1);
    infoNLP.zu(end-4:end-2) = 2*ones(3,1);;
else if all(s==(0&&1))
    infoNLP.zl(end-4:end-2) = [0;0;0];
    infoNLP.zu(end-4:end-2) = [0;0;0];
end

end