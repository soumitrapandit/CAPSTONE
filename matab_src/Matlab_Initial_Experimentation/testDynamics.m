function f_dynamics = testDynamics()
%   BipolarBot with simplified dynamics
%
addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*
% Define symbolic variables for angles and their derivatives
q1 = SX.sym('q1'); q2 = SX.sym('q2');
dq1 = SX.sym('dq1'); dq2 = SX.sym('dq2');
d2q1 = SX.sym('d2q1'); d2q2 = SX.sym('d2q2');

% Define Torques
tau1 = SX.sym('tau1');
tau2 = SX.sym('tau2');

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
Tau = [tau1;tau2];

%Calculating M
m11 = m1* (l1^2) + m2 *(l1^2+ 2*l1*l2*cos(q2) + l2^2);
m12 = m2*(l1*l2*cos(q2)+ l2^2);
m21 = m2*(l1*l2*cos(q2)+ l2^2);
m22 = m2*l2^2;

M = [m11,m12;m21,m22];

%Caculating C
c11 = -m2*l1*l2*sin(q2)*(2*dq1*dq2+dq2^2);
c21 = m2*l1*l2*dq2^2*sin(q2);

C = [c11;c21];

%Calculating G
g11 = (m1+m2)*l1*g*cos(q1)+m2*g*l2*cos(q1+q2);
g21 = m2*g*l2*cos(q1+q2);
G = [g11;g21];

% Solve for the accelerations
d2q = (M) \ (Tau -C - G);

% Define the state vector and its time derivative
x = [q; dq];
u = Tau;
xdot = [dq; d2q];

% Create the dynamics function
f_dynamics = Function('f_dynamics', {x,u,params}, {xdot});
%param_values = [9.81; 1; 1; 0.5; 0.5; 1; 1; 0.1; 0.1];  % g, l1, l2, c1, c2, m1, m2, I1, I2
end
