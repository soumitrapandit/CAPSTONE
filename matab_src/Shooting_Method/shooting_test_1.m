%%READ ME FIRST:
% x0 = initial state (all zeros)
% ui = optimization vector

clc; clear; close all;
model = BipolarBot();

params = struct;
params.model = model;
params.ncontrols = 1;
params.nstates = 8;
params.N = 3; %here N is the number of controls "windows" we have
params.t_init = 0;
params.t_delta = 0.01;
params.t_final = 20;
params.torque_limit = 4;

%Desired Pos:
params.desired_pos = [1,1.5];

%Setup Time Variables:
tspan = params.t_init:params.t_delta:params.t_final;
ti = linspace(tspan(1), tspan(end), params.N)';

%Setup Initial Control Variables:
%We have two control inputs on every iteration.
%However, the second one is a time release thing. 
%So our initial controls will be:
ui = [zeros(params.N*params.ncontrols,1);params.t_final];

%Setup Initial state:
%u0 = [zeros(params.N*params.ncontrols,1);params.t_final];
x0 = zeros(8,1);

%And we need to set up bounds:
ub = [params.torque_limit*ones(params.N*params.ncontrols,1);params.t_final];
lb = [-params.torque_limit*ones(params.N*params.ncontrols,1);params.t_init];

%And then we need a control Indexing Function which takes in t, ti, ui and
%returns u

options = optimoptions('fmincon',...
    Algorithm='interior-point',...
    Display='iter-detailed', ...
    MaxFunctionEvaluations=6000,...
    MaxIterations=200);

ui = fmincon(@(ui)shooting(x0, tspan, ti, ui,params), ...
    ui, ...
    [], [], ... % A = [], b = []
    [], [], ... % Aeq = [], beq = []
    lb, ub, ...
    [], ...
    options);

% Solve ODE with optimized control
[t, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);

params.model.animate(t,x)


%Debugging
%[t, x] = ode45(@(t,x)params.model.dynamics(t, x, [-2,t-100]), tspan, x0);

 %params.model.animate(t,x)

%shooting(x0,tspan,ti,ui,params)


%We also need a residual function:
function res = shooting(x0,tspan,ti,ui,params)
    [~, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);
    xf = x(end,:)';
    pos = params.model.kinematics(xf);
    current_pos = pos(end,:);
    res = norm(params.desired_pos - current_pos);
end

%And then we need a control Indexing Function which takes in t, ti, ui and
%returns u
function u = control(t,ti,ui)
    tau = ui(sum(t>=ti));
    a = t-ui(end,:);
    u = [tau;a];
end



