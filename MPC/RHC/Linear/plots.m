function plots(tout,X,tref,ref)
    figure(1)
    plot(tout, (180/pi)*X(:,1), 'LineWidth',1.5);
    hold on;
    plot(tref(1:40), (180/pi)*ref(1:40, 1), 'LineWidth',1.5);
    hold on;
    plot(tout, 27.5*ones(size(tout)), '--k');
    hold on;
    plot(tout, -27.5*ones(size(tout)), '--k');
    legend({'Generated', 'Reference', 'Constraint'}, 'Location', 'southwest', 'FontSize', 11);
    title('Linear Receding Horizon Control: Elevation', 'FontSize', 16);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Elevation (degrees)', 'FontSize', 14);
    grid on;

    figure(2)
    plot(tout, (180/pi)*X(:,2), 'LineWidth',1.5);
    hold on;
    plot(tref(1:40), (180/pi)*ref(1:40, 2), 'LineWidth',1.5);
    hold on;
    plot(tout, 90*ones(size(tout)), '--k');
    hold on;
    plot(tout, -90*ones(size(tout)), '--k');
    legend({'Generated', 'Reference', 'Constraint'}, 'Location', 'northwest', 'FontSize', 11);
    title('Linear Receding Horizon Control: Pitch', 'FontSize', 16);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Pitch (degrees)', 'FontSize', 14);
    grid on;

    figure(3)
    plot(tout, (180/pi)*X(:,3), 'LineWidth',1.5);
    hold on;
    plot(tref(1:40), (180/pi)*ref(1:40, 3), 'LineWidth',1.5);
    legend({'Generated', 'Reference'}, 'Location', 'northwest', 'FontSize', 11);
    title('Linear Receding Horizon Control: Travel', 'FontSize', 16);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Travel (degrees)', 'FontSize', 14);
    grid on;
    
    figure(4)
    plot(tout, (180/pi)*X(:,4), 'LineWidth',1.5);
    hold on;
    plot(tref(1:40), (180/pi)*ref(1:40, 4), 'LineWidth',1.5);
    legend({'Generated', 'Reference'}, 'Location', 'northwest', 'FontSize', 11);
    title('Linear Receding Horizon Control: Elevation Rate', 'FontSize', 14.6);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Elevation Rate (degrees/s)', 'FontSize', 14);
    grid on;
    
    figure(5)
    plot(tout, (180/pi)*X(:,5), 'LineWidth',1.5);
    hold on;
    plot(tref(1:40), (180/pi)*ref(1:40, 5), 'LineWidth',1.5);
    legend({'Generated', 'Reference'}, 'Location', 'northwest', 'FontSize', 11);
    title('Linear Receding Horizon Control: Pitch Rate', 'FontSize', 16);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Pitch Rate (degrees/s)', 'FontSize', 14);
    grid on;
    
    figure(6)
    plot(tout, (180/pi)*X(:,6), 'LineWidth',1.5);
    hold on;
    plot(tref(1:40), (180/pi)*ref(1:40, 6), 'LineWidth',1.5);
    legend({'Generated', 'Reference'}, 'Location', 'northwest', 'FontSize', 11);
    title('Linear Receding Horizon Control: Travel Rate', 'FontSize', 16);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Travel Rate (degrees/s)', 'FontSize', 14);
    grid on;
end