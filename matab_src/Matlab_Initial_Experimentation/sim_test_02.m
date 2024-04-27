
% Define parameter values
param_values = [9.81; 1; 1; 0.5; 0.5; 1; 1; 0.1; 0.1];

% Initial conditions [q1, q2, dq1, dq2]
y0 = [pi/2; pi/4; 0; 0];

% Time span
tspan = [0 10];

% Run ode45
[t, Y] = ode45(@(t, y) ode_system(t, y, param_values), tspan, y0);

% Plot results
figure;
plot(t, Y(:, 1), 'r', t, Y(:, 2), 'b');
xlabel('Time (s)');
ylabel('Angle (rad)');
title

function dy = ode_system(t, y, params)
    q = y(1:2);
    dq = y(3:4);
    % Compute dynamics
    out = f_dynamics(q, dq, params);
    M = full(out{1});
    Cdq_plus_G = full(out{2});
    d2q = M \ (-Cdq_plus_G);  % Solve for acceleration
    dy = [dq; d2q];  % Return derivatives
end
