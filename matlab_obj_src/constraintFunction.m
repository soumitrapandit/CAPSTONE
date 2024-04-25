function [c,ceq] = constraintFunction(t, t0, x0, z, p)
%There should be N-1 contraints for N states
%No Inequality Contraints
c = [];

%We can define the equality contraints using the robot dynamics
%ceq =  next_state - (current_state + delta_t * dynamics(current_state));

base_index = p.N*p.nstates;

ceq = zeros(base_index,1);

t_pre = t0;
x_pre = x0;

for k = 1:p.N

    state_index = p.nstates*(k-1)+1 : p.nstates*k;

    t_cur = t(k);
    x_cur = z(state_index);
    u_pre = [z(base_index+k); t0 - z(end)];
    f_pre = p.model.dynamics(t_pre, x_pre, u_pre);
    ceq(state_index,1) = x_cur - x_pre - f_pre*(t_cur - t_pre);

    t_pre = t_cur;
    x_pre = x_cur;
end

end