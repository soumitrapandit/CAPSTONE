%Let's define a constraint function as well
function [c,ceq] = constraints(x0,tspan,ti,ui,params)
    %Inequality Constraints: []
    c = [];

    % Equality Constraints: 

    [~, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);
    xf = x(end,:)';
    pos = params.model.kinematics(xf);
    current_pos = pos(end,:);
    ceq = norm(params.desired_pos - current_pos);
end