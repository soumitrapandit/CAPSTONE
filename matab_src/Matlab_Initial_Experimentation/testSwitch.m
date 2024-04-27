clear all
close all
clc

addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*
% Define the state and parameter symbols (re-definition if not in scope)
x = SX.sym('x', 4);% Assuming the state vector x has 4 elements [q1; q2; dq1; dq2]
u = SX.sym('u',2);% Assuming two control variables [tau1; tau2]
params = SX.sym('params', 9);  % Assuming 9 parameters [g; l1; l2; c1; c2; m1; m2; I1; I2]

% Create the dynamics function 
%f_dynamics = testDynamics();
f_dynamics = symbolicDynamics();

% Integrator options
opts = struct;
opts.tf = 0.01;  % final time for one step of integration

% Create the integrator
integrator = integrator('integrator', 'cvodes', struct('x', x, 'p', [u;params], 'ode', f_dynamics(x,u, params)), opts);
% Initial conditions and parameters
x0 = [0; 0; 0; 0];  % Example initial conditions
param_values = [9.81; 1; 1; 0.5; 0.5; 1; 1; 0.1; 0.1];  % Example parameter values
u0 = [0; 0];        % Current control inputs

%drawDoublePendulum(x0,0.5,0.5)

% Time setup
T_final = 10; % Total simulation time in seconds
dt = 0.001; % Time step defined in integrator options
N = floor(T_final/dt); % Number of time steps
times = linspace(0, T_final, N+1);
states = zeros(4, N+1);
states(:, 1) = x0; % Store initial condition
predefined_state = [0,0];
origin = [0,0];
l1 = 1; % Length of the first pendulum arm
l2 = 1; % Length of the second pendulum arm
% Simulate the system
figure;
for k = 1:N
    %Plot first
    clf
    theta1 = states(1,k);
    theta2 = states(2,k);
    % Calculate the position of the first joint
    joint1 = origin + [l1 * sin(theta1), l1 * cos(theta1)];

    % Calculate the position of the end of the second arm
    endEffector = joint1 + [l2 * sin(theta1 + theta2), l2 * cos(theta1 + theta2)];
    
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

    %Next step Simulation:
    sol = integrator('x0', states(:, k), 'p', [u0;param_values]);
    states(:, k+1) = full(sol.xf);  % Extract final state
    if norm(states(1:2,k+1)-predefined_state,2)<1e-2
        disp("Found")
        old_theta1 = states(1,k+1);
        old_theta2 = states(2,k+1);
        new_theta1 = 2*pi-(old_theta1+old_theta2);
        new_theta2 = pi - old_theta2;
        x_switch = [new_theta1,new_theta2,0,0];
        states(:,k+1) = x_switch;
        new_y = l1*sin(old_theta1)+l2*sin(old_theta1+old_theta2);
        new_x = l1*cos(old_theta1)+l2*cos(old_theta1+old_theta2);
        origin = [new_x,new_y];
    end
    
end
% Animate the double pendulum

%animateBipolarBot(times, states', l1, l2);
%drawBipolarBot(dt,states,l1,l2)