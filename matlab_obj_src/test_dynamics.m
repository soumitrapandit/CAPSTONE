clc; clear all; close all;

% syms x y q1 q2 dx dy dq1 dq2 'real';

syms g l1 l2 m1 m2 c1 c2 I1 I2 'positive';

q = sym('q%d',[4,1],'real');
dq = sym('dq%d',[4,1],'real');
d2q = sym('d2q%d',[4,1],'real');

[p, dp] = kinematics([q;dq],[c1 c2],[l1 l2]);

K = m1*dp(2,:)*dp(2,:)' + m2*dp(4,:)*dp(4,:)' + I1*dq(3)^2 + I2*(dq(3) + dq(4))^2;
K = K/2;

U = g*(m1*p(2,2) + m2*p(4,2));


L = K - U;

dL_dq = gradient(L,dq);

eom = jacobian(dL_dq,[q;dq])*[dq;d2q] - gradient(L,q);




M = simplify(jacobian(jacobian(K,dq),dq));
gr = simplify(gradient(U, q));
h = simplify(jacobian(jacobian(K,dq),q)*dq + gradient(K,q));

% %Calculate M,C,G
% obj.M = matlabFunction(jacobian(jacobian(K,dq),dq), Vars=[q1, q2]);
% obj.gr = matlabFunction(gradient(U, q), Vars=[q1, q2]);
% obj.h = matlabFunction(jacobian(jacobian(K,dq),q)*dq + gradient(K,q), Vars=[q1, q2, dx, dy, dq1, dq2]);


function [p, dp] = kinematics(x,c,l)
    p(1,:) = x(1:2,1)';
    p(2,:) = p(1,:) + c(1)*[cos(x(3)), sin(x(3))];
    p(3,:) = p(1,:) + l(1)*[cos(x(3)), sin(x(3))];
    p(4,:) = p(3,:) + c(2)*[cos(x(3) + x(4)), sin(x(3) + x(4))];
    p(5,:) = p(3,:) + l(2)*[cos(x(3) + x(4)), sin(x(3) + x(4))];

    dp(1,:) = x(5:6,1)';
    dp(2,:) = dp(1,:) + c(1)*x(7)*[-sin(x(3)), cos(x(3))];
    dp(3,:) = dp(1,:) + l(1)*x(7)*[-sin(x(3)), cos(x(3))];
    dp(4,:) = dp(3,:) + c(2)*(x(7) + x(8))*[-sin(x(3) + x(4)), cos(x(3) + x(4))];
    dp(5,:) = dp(3,:) + l(2)*(x(7) + x(8))*[-sin(x(3) + x(4)), cos(x(3) + x(4))];
end