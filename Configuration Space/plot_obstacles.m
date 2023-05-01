function plot_obstacles(obstacles)
    % Plot the obstacles
    figure;
    xlim([0 10]);
    ylim([0 10]);
    axis square;
    hold on;
    for i = 1:length(obstacles)
        plot(obstacles(i), 'FaceColor', 'k');
    end
end