clear all
close all
clc

addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

import casadi.*

% Define symbols
q1 = SX.sym('q1'); q2 = SX.sym('q2');
dq1 = SX.sym('dq1'); dq2 = SX.sym('dq2');
d2q1 = SX.sym('d2q1'); d2q2 = SX.sym('d2q2');

g = SX.sym('g', 1); l1 = SX.sym('l1', 1); l2 = SX.sym('l2', 1);
c1 = SX.sym('c1', 1); c2 = SX.sym('c2', 1);
m1 = SX.sym('m1', 1); m2 = SX.sym('m2', 1);
I1 = SX.sym('I1', 1); I2 = SX.sym('I2', 1);

q = [q1; q2];
dq = [dq1; dq2];
d2q = [d2q1; d2q2];

% Position vectors
p1 = c1 * [cos(q1); sin(q1)];
pj = l1 * [cos(q1); sin(q1)];
p2 = pj + c2 * [cos(q1 + q2); sin(q1 + q2)];

% Velocity vectors
v1 = jacobian(p1, q) * dq;
v2 = jacobian(p2, q) * dq;

% Kinetic and potential energy
K = (m1 * (v1' * v1) + m2 * (v2' * v2) + I1 * dq1^2 + I2 * (dq1 + dq2)^2) / 2;
U = m1 * g * p1(2) + m2 * g * p2(2);

% Lagrangian
L = K - U;

% Derivative of Lagrangian
Ldq = jacobian(L, dq).';
eom = jacobian(Ldq, q) * dq + jacobian(Ldq, dq) * d2q - gradient(L, q);

% Mass matrix, Coriolis and Gravity terms
M = jacobian(eom, d2q);
G = gradient(U, q);
C = jacobian(eom - M * d2q - G, dq) / 2;

% Check equations
check = eom - (M * d2q + C * dq + G);

% Display check
disp('||check|| should be zero:');
disp(full(norm(check)));

