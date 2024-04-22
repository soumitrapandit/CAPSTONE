clear
close all
clc
myBot = BipolarBot();

% Test 1: 
% Case : Robot is Attached
% u = [0,0];
% [t,x] = ode45(@(t,x)myBot.dynamics(t,x,u),[0, 10], zeros(8,1));
%animateBipolarBot(t,x)

% Test 2:
% Case : Robot is Detached
% u = [0,1];
% [t,x] = ode45(@(t,x)myBot.dynamics(t,x,u),[0, 10], zeros(8,1));
%animateBipolarBot(t,x)