clear all
pareto = table2array(importpareto('pareto_data_RHC.txt', 2, 935));

pareto = sortrows(pareto,1);
pareto(580:end,:) = [];
front = [];

ymin = pareto(1,2);
for i = 1:length(pareto)
    if pareto(i,2) < ymin;
        front = [front; pareto(i,:)];
        ymin = pareto(i,2);
    end
end

scatter(pareto(:,1),pareto(:,2));
hold on;
scatter(front(:,1),front(:,2), 50, 'filled');
hold on;
stairs(front(:,1), front(:,2),'r', 'LineWidth',1.5)
title('Pareto Front for Linear Receding Horizon Controller', 'FontSize', 14)
yticks(0:10:140);
xlabel('RMS Error', 'FontSize', 14)
ylabel('Computation Time', 'FontSize', 14)
legend({'Simulation Point', 'Pareto Point', 'Pareto Front'}, 'Location', 'northeast', 'FontSize', 11);
grid on;


