clear
close all
clc

addpath('C:\Users\smtrp\OneDrive\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

%Simulation Parameters
T = 0.01;
N = 3; %Horizon Window
 
%Tau Limits
tau1_min = 0; tau1_max = 0;
tau2_min = -400; tau2_max = 400; %What are the units associated with this?
%What would be a good tau constraint?
%Question for later, I suppose

%State Variables
theta1 = SX.sym('theta1');
theta2 = SX.sym('theta2');
theta1_dot = SX.sym('theta1_dot');
theta2_dot = SX.sym('theta2_dot');

states = [theta1,theta2,theta1_dot,theta2_dot];
n_states = length(states);

%Controls
tau1 = SX.sym('tau1');
tau2 = SX.sym('tau2');

%Contorl Initialization
controls = [tau1,tau2];
n_controls = length(controls);

%This is our custom defined
f_dynamics = symbolicDynamics();
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

X(:,1) = P(1:n_states); %Initial State
param_values = [9.81; 1; 1; 0.5; 0.5; 1; 1; 0.1; 0.1];
for k = 1:N
    current_state = X(:,k);
    current_control = U(:,k);
    current_dynamics = f_dynamics(current_state,current_control,param_values);
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
Q = zeros(4,4); Q(1,1) = 1; Q(2,2) = 5; Q(3,3) = 0.1; Q(4,4) = 0.5;

%Defining Costs Associated with Controls
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05;

for k = 1:N
    current_state = X(:,k); 
    current_control = U(:,k);
    %MPC minimization equation:
    state_error = (current_state - P(5:8));
    obj = obj + state_error'*Q*state_error + current_control'*R*current_control;
end

% compute constraints
%Do I need these constraints. I don't think so.
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state theta1
    g = [g ; X(2,k)];   %state theta2
    % g = [g ; X(3,k)];   %state theta1_dot
    % g = [g ; X(4,k)];   %state theta2_dot
end

% make the decision variables one column vector
OPT_variables = reshape(U,2*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g , 'p', P);

%This is from the casadia suite. Don't change. 
opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% inequality constraints (state constraints)
args = struct;
args.lbg = -2*pi;  % lower bound of the states theta1 and theta2
args.ubg = 2*pi;   % upper bound of the states theta1 and theta2 

% input constraints
args.lbx(1:2:2*N-1,1) = tau1_min; args.lbx(2:2:2*N,1)   = tau2_min;
args.ubx(1:2:2*N-1,1) = tau1_max; args.ubx(2:2:2*N,1)   = tau2_max;


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP

% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = [0 ; 0 ; 0 ; 0];    % Start at rest.
xs = [0 ; pi/2 ; 0 ; 0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);  % two control inputs 

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];


% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
error_values = [];
while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / T)
    current_error = norm((x0-xs),2);
    error_values = [error_values, current_error];
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',2*N,1); % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',2,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:4,mpciter+1)= full(ff_value)';
    u_cl= [u_cl ; u(1,:)];

    [t0, x0, u0] = next(T, t0, x0, u,f_dynamics,param_values); % get the initialization of the next optimization step
    t(mpciter+1) = t0;

    xx(:,mpciter+2) = x0;  
    mpciter;
    mpciter = mpciter + 1;
    
end
main_loop_time = toc(main_loop)

ss_error = norm((x0-xs),2);
% Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,rob_diam) % a drawing function
% Plotting the error after the loop
figure;
plot(error_values, 'LineWidth', 2);
title('MPC Tracking Error');
xlabel('Iteration Number');
ylabel('Error Norm');
grid on;

%animateBipolarBot(sim_tim,xx,1,1)

