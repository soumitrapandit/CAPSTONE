function drawBipolarBot(delta_t,state,l1,l2)
% Extract angles from the state
n = size(state);
figure;
for k = 1:n(2)
    clf
    theta1 = state(1,k); % Angle of the first pendulum arm
    theta2 = state(2,k); % Angle of the second pendulum arm
    
    % Define the origin
    origin = [0, 0]; 
    
    % Calculate the position of the first joint
    joint1 = origin + [l1 * sin(theta1), l1 * cos(theta1)];

    % Calculate the position of the end of the second arm
    endEffector = joint1 + [l2 * sin(theta1 + theta2), l2 * cos(theta1 + theta2)];
    
    % Open a new figure
    
    hold on;
    axis equal;
    grid on;
    title('BiPolar Bot');
    xlabel('X position');
    ylabel('Y position');
    
    % Plot the first link
    plot([origin(2), joint1(2)], [origin(1), joint1(1)], 'k-', 'LineWidth', 2);
    
    % Plot the second link
    plot([joint1(2), endEffector(2)], [joint1(1), endEffector(1)], 'r-', 'LineWidth', 2);
    
    % Plot the joints
    plot(origin(2), origin(1), 'ko', 'MarkerFaceColor', 'k'); % Origin joint
    plot(joint1(2), joint1(1), 'ko', 'MarkerFaceColor', 'k'); % First joint
    plot(endEffector(2), endEffector(1), 'ro', 'MarkerFaceColor', 'r'); % End effector
    % Set the axes limits
    xlim([-l1 - l2, l1 + l2] * 1.1);
    ylim([-l1 - l2, l1 + l2] * 1.1);
    drawnow
end
end