%We also need a residual function:
function res = shooting(x0,tspan,ti,ui,params)
    [~, x] = ode45(@(t,x)params.model.dynamics(t, x, control(t, ti, ui)), tspan, x0);
    xf = x(end,:)';
    pos = params.model.kinematics(xf);
    current_pos = pos(end,:);
    res = norm(params.desired_pos - current_pos);% + norm(xf(5:8));
    %res = norm(params.desired_pos - [0 0 0 0 1]*params.model.kinematics(x(end,:)'));
end
