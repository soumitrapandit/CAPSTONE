clear all
close all
clc
addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

x0 = [pi/4; -pi/4; 0; 0];  % Initial state [q1, q2, dq1, dq2]
param_values = [9.81; 1; 1; 0.5; 0.5; 1; 1; 0.1; 0.1];  % g, l1, l2, c1, c2, m1, m2, I1, I2
t = 0;

dynamics = dynamicsBipolarBot(t,x0,param_values)
f_dynamics = symbolicDynamics()