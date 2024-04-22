clear all
close all
clc

addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

%Simulation Parameters
T = 0.2;
N = 3; %Horizon Window
rob_dim = 0.3;
 
%Velocity limits
v_max = 0.6; v_min = -v_max;
omega_max = 0.4; omega_min = -omega_max;

%State Variables
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
states = [x;y;theta];
n_states = length(states);

%velocities,linear and angular
v = SX.sym('v');
omega = SX.sym('omega');

%Contorl Initialization
controls = [v;omega];
n_controls = length(controls);

%System Dynamics
dynamics = [v*cos(theta);v*sin(theta);omega];

f = Function('f',{states,controls},{dynamics});
%U is the variable which stores the control actions for
% each step in the horizon
U = SX.sym('U',n_controls,N); % RxC = n_controls x N

%P is the variable that stores two things
%The first n_states store the initial state
%The last n_states store the reference state
P = SX.sym('P',n_states+n_states);

%The prediction space
%OR the state evolution space
X = SX.sym('X',n_states,(N+1));

X(:,1) = P(1:3); %Initial State

for k = 1:N
    current_state = X(:,k);
    current_control = U(:,k);
    current_dynamics = f(current_state,current_control);
    next_state = current_state + T*current_dynamics;
    X(:,k+1) = next_state;
end

%Now, this is the part where the magic happens
%We're going to use the symbolic equation of X
%To get a Prdiction
ff = Function('ff',{U,P},{X});

%ff basically computes the Optimal Trajectory once
%the Optimal Control is known.

%Let's Calculate the Objective Function Now.
%The objective function will also be given as a 
% symbolic summation
obj = 0;
g = []; %Constraints

%Defining Costs Associated with State
%Q has some specifications that have to be respected
Q = zeros(3,3); Q(1,1) = 1; Q(2,2) = 5; Q(3,3) = 0.1;

%Defining Costs Associated with Controls
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05;

for k = 1:N
    current_state = X(:,k); current_control = U(:,k);
    %MPC minimization equation:
    state_error = (current_state - P(4:6));
    obj = obj + state_error'*Q*state_error + current_control'*R*current_control;
end

%Defining Constraints
%We're going to define the constraints for every single contol input
%We're also going to use the Dynamics as equality contraints

for k = 1:N+1
    g = [g;X(1,k)];
    g = [g;X(2,k)];
end
%I am not sure about the last part. What are we doing here exactly?

%Make the decision variable a column vector.
OPT_variables = reshape(U,2*N,1);
nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);

%
opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level = 0; %0-3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver','ipopt',nlp_prob,opts);

%Arguments
args = struct;
args.lbg = -2; %Lower Bound of StateSpace
args.ubg = 2; %Upper Bound of StateSpace

%Argument Constraints:
args.lbx(1:2:2*N-1) = v_min; args.ubx(2:2:2*N) = omega_min;
args.lby(1:2:2*N-1) = v_max; args.uby(2:2:2*N) = omega_max;

%-----------------------------------------------------------
%SIMULATION LOOP
%-----------------------------------------------------------

t0 = 0;
x0 = [0;0;0.0]; %Initial Starting Point
xs = [1.5;1.5;0]; %Desired State

xx(:,1) = x0; %Contains History of States
t(1) = t0; 
u0 = zeros(N,2); %Initial Control
sim_time = 20; %Simulation Time

%Start MPC
mpciter = 0;
xxl = [];
u_cl = [];

while (norm((x0-xs),2)) > 1e-2 && mpciter <sim_time/T
    args.p = [x0,xs];
    args.x0 = reshape(u0',2*N,1); %Initialize Optimization Variables
    %DoNOT confuse this x0 with our initial state. This x0 is the
    %Vector of initial optimization variables. We name it x0 as that's
    %What casadi uses. 
    sol = solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,...
        'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
    u = reshape(full(sol.x)',2,N);
    ff_value = ff(u',args.p); %compute Optimal Control for Trajectory
    xxl(:,1:3,mpciter+1) = full(ff_value)';
    u_cl = [u_cl; u(1,:)];
    [t0,x0,u0] = shift(T,t0,x0,u,f);
    t(mpciter+1) = t0;
    xx(:,mpciter+2) = x0;
    mpciter = mpciter + 1;
end;

main_loop_time = toc(main_loop)
ss_error = norm((x0-xs),2)
Draw_MPC_point_stabilization_v1(t,xx,xxl,u_cl,xs,N, robot_diam)









