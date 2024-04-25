%Okay so I basically have to use fmincon to solve this.
%Ideally, this would be done with an object and perhaps, eventually, I will
%build up to that. But for now, let's just try to build something that
%works. 

%Lets Build the Z vector:
N = 3;
nstates = 8;
ncontrols = 1;
torque_limit = 6;
u_lb = -torque_limit    ;
u_ub = torque_limit;

%Initial Guess:
Z0 = zeros(N*(nstates+ncontrols)+1);

%Time Constraints:
t_init = 0;
t_final = 4;
delta_t = 0.01;
t_vec = t_init:delta_t:t_final;

%Lets create the Z vectors


%fmincon specific variables:
A = [];
b = [];

Aeq = [];
beq = [];

%Lets create the bounds
state_lb = -inf(N*nstates,1);
state_ub = inf(N*nstates,1);
controls_lb = u_lb*ones(N*ncontrols,1);
controls_ub = u_ub*ones(N*ncontrols,1);
alpha_lb = t_init;
alpha_ub = t_final;

lb = [state_lb;controls_lb;alpha_lb];
ub = [state_ub;controls_ub;alpha_ub];

%fmincon
%x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)

%x = fmincon(@objectiveFunction,zeros(10,1),Aeq,beq,lb,ub,@constraintFunction);
