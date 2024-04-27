%Okay let's try to solve this.
%Let's prep this problem.
%What do I need? Probably an initial guess
%I would also need the vector of controls to be optimized
%And I would need the bounds for the controls
%Most importantly, I would need the time vector

clc; clear; close all;

params.model = BipolarBot();
params = struct;
params.ncontrols = 1;
params.nstates = 8;
params.N = 3; %here N is the number of controls "windows" we have
params.t_init = 0;
params.t_delta = 0.01;
params.t_final = 2;
params.torque_limit = 4;

%Desired Pos:
desired_pos = [1,1.5];

%Setup Time Variables:
tspan = params.t_init:params.t_delta:params.t_final;
ti = linspace(tspan(1), tspan(end), params.N)';

%Setup Initial Control Variables:
%We have two control inputs on every iteration.
%However, the second one is a time release thing. 
%So our initial controls will be:
ui = [zeros(params.N*params.ncontrols,1);params.t_final];

%Setup Initial State:
x0 = zeros(params.nstates);

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

%ui = fmincon(@(ui)shooting(x0, tspan, ti, ui,desired_pos), ...
%     ui, ...
%     [], [], ... % A = [], b = []
%     [], [], ... % Aeq = [], beq = []
%     lb, ub, ...
%     [], ...
%     options);
% 
% % Solve ODE with optimized control
% [t, x] = ode45(@(t,x)dynamics(t, x, control(t, ti, ui)), tspan, x0);


%Debugging
[~, x] = ode45(@(t,x)params.model.dynamics(t, x, [-2,t-100]), tspan, x0);


%We also need a residual function:
function res = shooting(x0,tspan,ti,ui)
    [~, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);
    xf = x(end,:);
    res = norm(params.desired_pos - params.model.kinematics(xf));
end

%And then we need a control Indexing Function which takes in t, ti, ui and
%returns u
function u = control(t,ti,ui)
    tau = ui(sum(t>=ti));
    a = t-ui(end,:);
    u = [tau;a];
    
end



