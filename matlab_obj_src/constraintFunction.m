function [c,ceq] = constraintFunction(t,Z)
%There should be N-1 contraints for N states
%No Inequality Contraints
c = [];

%We can define the equality contraints using the robot dynamics
%ceq =  next_state - (current_state + delta_t * dynamics(current_state));
    nstates = 8;
    ncontrols = 2;
    N = size(Z)/(nstates+ncontrols);
    myBot = BipolarBot();
    iter_step = nstates+ncontrols;

    for i = 1:iter_step:N*step_iter
        
        current_state = Z(i:i+nstates);
        current_controls = Z(i+nstates+1:i+nstates+ncontrols);
        current_dynamics = myBot.dynamics(t,current_state,current_controls);

        next_state = Z(i+iter_step:i+iter_step+nstates);

        new_constraint = next_state - (current_state + delta_t * current_dynamics);

        ceq = [ceq;new_constraint];
    end

end