clear all
close all
clc

%-------------------Initialize-------------------------
syms x y q1 q2 dx dy dq1 dq2 d2x d2y d2q1 d2q2 'real';
syms tau1 tau2 alpha 'real';
l1 = 1;
l2 = 1;
m1 = 1;
m2 = 1;
I1 = 0.1;
I2 = 0.1;
c1 = 0.5;
c2 = 0.5;
g = 9.8;
%------------------------------------------------------

pos = [x;y];
vel = [dx;dy];
acc = [d2x;d2y]; 

q = [x; y; q1;q2];
dq = [dx; dy; dq1; dq2];
d2q = [d2x; d2y; d2q1; d2q2];


p1 = pos + c1*[cos(q1); sin(q1)];
pj = pos + l1*[cos(q1); sin(q1)];
p2 = pj + c2*[cos(q1 + q2); sin(q1  + q2)];

% v1 = vel + c1*[-sin(q1); cos(q1)]*dq1;
% v2 = vel + l1*[-sin(q1);cos(q1)]*dq1 + ...
%     c2*l2*[-sin(q1+q2);cos(q1+q2)]*(dq1+dq2);

v1 = simplify(jacobian(p1,obj.q)*obj.dq);
v2 = simplify(jacobian(p2,obj.q)*obj.dq);
%Calculate Kinetic Energy
K = ( m1*(v1'*v1) + m2*(v2'*v2) + I1*dq1^2 + I2*(dq1 + dq2)^2)/2;

%Calculate Potential Energy
U = m1*g*p1(2) + m2*g*p2(2);

%Calculate the Lagrangian
L = K - U;
Ldq = gradient(L,dq);
eom = simplify(jacobian(Ldq,q)*dq + jacobian(Ldq,dq)*d2q - gradient(L,q));

%Calculate M,C,G
M = simplify(jacobian(eom, d2q));
G = simplify(gradient(U,q));
C = simplify(jacobian(eom - M*d2q - G,dq))/2;

%Formulate Equation
tau = [0;0;tau1;tau2];
% Solve for the accelerations
d2q = (M) \ (tau-C * dq - G);
dynamics = [dq;d2q];
%val = subs(dynamics,[x,y,q1,q2,dx,dy,dq1,dq2,tau1,tau2],[0,0,pi/2,pi/2,0,0,0,0,0,0])
size([q;dq;tau1;tau2])
size([0;0;p1/2;pi/2;0;0;0;0;0;0])
val2 = subs(dynamics,[q;dq;tau1;tau2],[0;0;pi/2;pi/2;0;0;0;0;0;0])
tau(3:4)
