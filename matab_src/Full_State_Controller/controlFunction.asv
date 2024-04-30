function z = controlFunction(current_state, desired_pos, params)
%Initial State
x_init = current_state;

%Initial Guess:
z0 = zeros(params.N*(params.nstates+params.ncontrols),1);
z0 = [z0; params.t_final];

%fmincon specific variables:
A = [];
b = [];

Aeq = [];
beq = [];

%Lets create the bounds
state_lb = -inf(params.N*params.nstates,1);
state_ub = inf(params.N*params.nstates,1);
controls_lb = -params.torque_limit*ones(params.N*params.ncontrols,1);
controls_ub = params.torque_limit*ones(params.N*params.ncontrols,1);
alpha_lb = params.t_init;
alpha_ub = params.t_final;

lb = [state_lb; controls_lb; alpha_lb];
ub = [state_ub; controls_ub; alpha_ub];

options = optimoptions('fmincon',...
    Algorithm='interior-point',...
    Display='iter-detailed', ...
    MaxFunctionEvaluations=6000,...
    MaxIterations=200);

%fmincon
z = fmincon(@(z)objectiveFunction(params.t, z, desired_pos, params), ...
    z0, ...
    A, b, ...
    Aeq,beq, ...
    lb,ub, ...
    @(z)constraintFunction(params.t, params.t_init, x_init, z, params), ...
    options);
end