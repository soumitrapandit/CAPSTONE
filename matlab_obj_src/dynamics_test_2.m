clear
close all
clc
myBot = BipolarBot();


[t,x] = ode45(@(t,x)myBot.dynamics(t,x,[0,1]),[0, 10], zeros(8,1));
size(x)
animateBipolarBot(t,x)