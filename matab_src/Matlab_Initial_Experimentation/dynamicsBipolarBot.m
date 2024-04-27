function dynamics = dynamicsBipolarBot(t,state,param_values)
addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*
% Define symbolic variables for angles and their derivatives
q1 = SX.sym('q1'); q2 = SX.sym('q2');
dq1 = SX.sym('dq1'); dq2 = SX.sym('dq2');
d2q1 = SX.sym('d2q1'); d2q2 = SX.sym('d2q2');

% Define parameters as symbols
g = SX.sym('g'); % Gravitational acceleration
l1 = SX.sym('l1'); l2 = SX.sym('l2'); % Lengths of the pendulum arms
c1 = SX.sym('c1'); c2 = SX.sym('c2'); % Centers of mass
m1 = SX.sym('m1'); m2 = SX.sym('m2'); % Masses
I1 = SX.sym('I1'); I2 = SX.sym('I2'); % Moments of inertia

% Group all states and parameters
q = [q1; q2];
dq = [dq1; dq2];
d2q = [d2q1; d2q2];
params = [g; l1; l2; c1; c2; m1; m2; I1; I2]; % Parameter vector

% Position vectors of centers of mass
p1 = c1 * [cos(q1); sin(q1)];
pj = l1 * [cos(q1); sin(q1)];
p2 = pj + c2 * [cos(q1 + q2); sin(q1 + q2)];

% Velocities of centers of mass
v1 = jacobian(p1, q) * dq;
v2 = jacobian(p2, q) * dq;

% Kinetic energy
K = (m1 * (v1' * v1) + m2 * (v2' * v2) + I1 * dq1^2 + I2 * (dq1 + dq2)^2) / 2;

% Potential energy
U = m1 * g * p1(2) + m2 * g * p2(2);

% Lagrangian
L = K - U;

% Euler-Lagrange equations
Ldq = jacobian(L, dq).';
eom = jacobian(Ldq, q) * dq + jacobian(Ldq, dq) * d2q - gradient(L, q);

% Solve for accelerations
M = jacobian(eom, d2q);
G = gradient(U, q);
C = jacobian(eom - M * d2q - G, dq) / 2;

% Solve for the accelerations
d2q = (M) \ (-C * [dq1; dq2] - G);

% Define the state vector and its time derivative
x = [q; dq];
xdot = [dq; d2q];

% Create the dynamics function
f_dynamics = Function('f_dynamics', {x, params}, {xdot});
%param_values = [9.81; 1; 1; 0.5; 0.5; 1; 1; 0.1; 0.1];  % g, l1, l2, c1, c2, m1, m2, I1, I2

% Call the dynamics function with the initial state and parameter values
dynamics= f_dynamics(state, param_values);
% Extract individual derivatives

end