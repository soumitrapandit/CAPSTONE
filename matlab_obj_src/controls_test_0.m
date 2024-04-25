clc; clear all; close all;

%Okay so I basically have to use fmincon to solve this.
%Ideally, this would be done with an object and perhaps, eventually, I will
%build up to that. But for now, let's just try to build something that
%works. 

%Lets Build the Z vector:

model = BipolarBot();

torque_limit = 5;

params = struct('model', model, ...
   'N',200,...
   'nstates', 8, ...
   'ncontrols', 1, ...
   'torque_limit', 6, ...
   'u_lb', -torque_limit, ...
   'u_ub', torque_limit,...
   'eps', 1e-3);

desired_pos = [1 0];

%Time Constraints:
t_init = 0;
t_final = 10;
t = linspace(t_init, t_final, params.N+1);
t0 = t(1);
t = t(2:end);

%Initial State
x_init = zeros(8,1);

%Initial Guess:
z0 = zeros(params.N*(params.nstates+params.ncontrols),1);
z0 = [z0; t_final];

%fmincon specific variables:
A = [];
b = [];

Aeq = [];
beq = [];

%Lets create the bounds
state_lb = -inf(params.N*params.nstates,1);
state_ub = inf(params.N*params.nstates,1);
controls_lb = -torque_limit*ones(params.N*params.ncontrols,1);
controls_ub = torque_limit*ones(params.N*params.ncontrols,1);
alpha_lb = t_init;
alpha_ub = t_final;

lb = [state_lb; controls_lb; alpha_lb];
ub = [state_ub; controls_ub; alpha_ub];

options = optimoptions('fmincon',...
    Algorithm='interior-point',...
    Display='iter-detailed', ...
    MaxFunctionEvaluations=6000,...
    MaxIterations=200);

%fmincon
z = fmincon(@(z)objectiveFunction(t, z, desired_pos, params), ...
    z0, ...
    A, b, ...
    Aeq,beq, ...
    lb,ub, ...
    @(z)constraintFunction(t, t_init, x_init, z, params), ...
    options);


x = reshape(z(1:params.N*params.nstates),params.nstates, params.N)';
plot(t,x);

model.animate(t,x);

