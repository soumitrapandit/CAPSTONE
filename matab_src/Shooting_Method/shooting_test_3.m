%%READ ME FIRST:
% x0 = initial state (all zeros)
% ui = optimization vector

clc; clear; close all;
model = BipolarBot();

params = struct;
params.model = model;
params.ncontrols = 1;
params.nstates = 8;
params.N = 10; %here N is the number of controls "windows" we have
params.t_init = 0;
params.t_delta = 0.01;
params.t_final = 10;
params.torque_limit = 1;

%Desired Pos:
params.desired_pos = [1, 0];

params.outer_options = optimoptions('ga',...
    Display='iter');
params.inner_options = optimoptions('fmincon',...
    Algorithm='interior-point',...
    Display='off', ...
    MaxFunctionEvaluations=6000,...
    MaxIterations=200);

x0 = zeros(8,1);


% p = [t_final, N]
intcon = 2; % second index in p is an integer
t_final_min = 2;
t_final_max = 7;
N_min = 2;
N_max = 10;

p = ga(@(p)outer_cost(p, params, x0), ...
        2, ...
        [], [], ... % A = [], b = []
        [], [], ... % Aeq = [], beq = []
        [t_final_min; N_min], [t_final_max; N_max], ...
        [], ...
        intcon);


params.t_final = p(1);
params.N = p(2);

tspan = [params.t_init, p(1)];
ti = linspace(tspan(1), tspan(end), p(2))';
ui = [zeros(p(2)*params.ncontrols,1); p(1)];

ub = [params.torque_limit*ones(p(2)*params.ncontrols,1);p(1)];
lb = [-params.torque_limit*ones(p(2)*params.ncontrols,1);params.t_init];
ui = fmincon(@(ui)inner_cost(x0, tspan, ti, ui, params), ...
        ui, ...
        [], [], ... % A = [], b = []
        [], [], ... % Aeq = [], beq = []
        lb, ub, ...
        [], ...
        params.inner_options);
[t, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);
% params.model.animate(t,x)
figure();
for k=1:length(t)
    p(k,:) = [0 0 0 0 1]*params.model.kinematics(x(k,:)');
end
plot(p(:,1),p(:,2),Color=lines(1),LineWidth=1);
hold on
plot(params.desired_pos(1), params.desired_pos(2), ...
    Marker="x", Color=[0 1]*lines(2), LineWidth=2);


function J = outer_cost(p, params, x0)
    % t_final = p(1);
    % N = p(2);
    tspan = [params.t_init, p(1)];
    ti = linspace(tspan(1), tspan(end), p(2))';
    ui = [zeros(p(2)*params.ncontrols,1); p(1)];

    ub = [params.torque_limit*ones(p(2)*params.ncontrols,1);p(1)];
    lb = [-params.torque_limit*ones(p(2)*params.ncontrols,1);params.t_init];

    [~, J] = fmincon(@(ui)inner_cost(x0, tspan, ti, ui, params), ...
        ui, ...
        [], [], ... % A = [], b = []
        [], [], ... % Aeq = [], beq = []
        lb, ub, ...
        [], ...
        params.inner_options);
end


function res = inner_cost(x0,tspan,ti,ui,params)
    [~, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);
    pos = params.model.kinematics(x(end,:)');
    current_pos = pos(end,:);
    res = norm(params.desired_pos - current_pos);
end

function u = control(t,ti,ui)
    tau = ui(sum(t>=ti));
    a = t-ui(end,:);
    u = [tau;a];
end