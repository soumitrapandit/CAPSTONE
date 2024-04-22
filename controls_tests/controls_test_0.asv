%Okay so I basically have to use fmincon to solve this.
%Ideally, this would be done with an object and perhaps, eventually, I will
%build up to that. But for now, let's just try to build something that
%works. 

%Lets Build the Z vector:

N = 3;

nstates = 8;

ncontrols = 2;

tf = 4; %4 seconds? Why 4 seconds?

%Lets create the Z vectors
%

Z = zeros(nstates*N + ncontrols*N,1);


%fmincon specific variables:
A = [];
b = [];

Aeq = [];
beq = [];

%Lets create the bounds
lb = [-inf(nstates,1);-4;0];

ub = [inf(nstates,1);4;tf];

%fmincon

%x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)

x = fmincon(@objectiveFunction,zeros(10,1),Aeq,beq,lb,ub,@constraintFunction);
