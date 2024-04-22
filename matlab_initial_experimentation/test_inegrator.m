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

% Simulate the system
for k = 1:N
    sol = integrator('x0', states(:, k), 'p', [u0;param_values]);
    states(:, k+1) = full(sol.xf);  % Extract final state
end
% Animate the double pendulum
l1 = 1; % Length of the first pendulum arm
l2 = 1; % Length of the second pendulum arm
%animateBipolarBot(times, states', l1, l2);
drawBipolarBot(dt,states,l1,l2)

