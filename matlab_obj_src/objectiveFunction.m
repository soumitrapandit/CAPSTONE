function t_opt =  objectiveFunction(t,Z)
%So we have all the states and the controls in this function
%Why should this even work when the Z vector is not populated?
%We're passing it as input to fmicon but isn't fmincon supposed to use it
%to choose the best z 
%Or am I missing something?
%Oh, I see what we're doing here.

%Initializations:
    nstates = 8;
    ncontrols = 2;
    N = size(Z)/(ncontrols+nstates);
    desired_pos = [1,1];
    atol = 1e-3;
    t_star = []; %The set of all the steps when cost is within atol
    myBot = BipolarBot();

%Construct Z from control inputs?
    step_iter = nstates+ncontrols;
    for i = 1:step_iter:N*step_iter
        current_state = Z(i:i+nstates);
        [p,dp] = myBot.kinematics(current_state);
        current_pos = p(5);
        if costFunction(current_pos,desired_pos) < atol
            t_star = [t_star;i];
        end
    end
    t_opt = min(t_star);

end