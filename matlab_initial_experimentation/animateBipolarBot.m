function animateBipolarBot(t, states, l1, l2)
    % Create figure
    figure;
    axis equal;
    hold on;
    xlim([-l1-l2, l1+l2]);
    ylim([-l1-l2, l1+l2]);
    xlabel('X Position');
    ylabel('Y Position');
    title('Double Pendulum Animation');

    % Extract angles from states
    theta1 = states(:, 1);
    theta2 = states(:, 2);

    % Calculate positions of the first and second pendulum
    y1 = l1 * sin(theta1);
    x1 = l1 * cos(theta1);
    y2 = y1 + l2 * sin(theta1 + theta2);
    x2 = x1 + l2 * cos(theta1 + theta2);

    % Plot initial positions
    p1 = plot([0, x1(1)], [0, y1(1)], 'r', 'LineWidth', 2); % First arm
    p2 = plot([x1(1), x2(1)], [y1(1), y2(1)], 'b', 'LineWidth', 2); % Second arm
    m1 = plot(x1(1), y1(1), 'ro', 'MarkerFaceColor', 'r'); % Mass at first pendulum
    m2 = plot(x2(1), y2(1), 'bo', 'MarkerFaceColor', 'b'); % Mass at second pendulum

    % Animation loop
    for k = 2:length(t)
        set(p1, 'XData', [0, x1(k)], 'YData', [0, y1(k)]);
        set(p2, 'XData', [x1(k), x2(k)], 'YData', [y1(k), y2(k)]);
        set(m1, 'XData', x1(k), 'YData', y1(k));
        set(m2, 'XData', x2(k), 'YData', y2(k));
        drawnow;
        pause(t(k) - t(k-1)); % Pause to match the simulation time step
    end
end
