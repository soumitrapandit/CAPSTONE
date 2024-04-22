clc; clear all; close all;

syms q1 q2 dq1 dq2 d2q1 d2q2 'real';
syms g l1 l2 c1 c2 m1 m2 I1 I2 'positive';
syms u1 u2 'real';

q = [q1; q2];
dq = [dq1; dq2];
d2q = [d2q1; d2q2];

p1 = c1*[cos(q1); sin(q1)];
pj = l1*[cos(q1); sin(q1)];
p2 = pj + c2*[cos(q1 + q2); sin(q1  + q2)];

v1 = simplify(jacobian(p1,q)*dq);
v2 = simplify(jacobian(p2,q)*dq);

K = ( m1*(v1'*v1) + m2*(v2'*v2) + I1*dq1^2 + I2*(dq1 + dq2)^2  )/2;
U = m1*g*p1(2) + m2*g*p2(2);

L = K - U;

Ldq = gradient(L,dq);

eom = simplify(jacobian(Ldq,q)*dq + jacobian(Ldq,dq)*d2q - gradient(L,q));


M = simplify(jacobian(eom, d2q));
G = simplify(gradient(U,q));
C = simplify(jacobian(eom - M*d2q - G,dq))/2;


check = eom - (M*d2q + C*dq + G);

fprintf("||check|| = %d, must be zero.\n", norm(simplify(check)));

