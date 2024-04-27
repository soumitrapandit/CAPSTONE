clear
close all
clc
myBot = BipolarBot();

x0 = zeros(8,1);

% Test 1: 
% Case : Robot is Detached
%u = [0,1];

% Test 2:
% Case : Robot is Attached
%u = [0,-1];

u = @(t,x) [-2*x(8); 2 < t & t < 2.2];


[t,x] = ode45(@(t,x)myBot.dynamics(t,x,u(t,x)),0:0.01:20, zeros(8,1));

myBot.animate(t, x);