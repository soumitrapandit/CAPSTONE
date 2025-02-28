clc; clear all; close all;

%Okay so I basically have to use fmincon to solve this.
%Ideally, this would be done with an object and perhaps, eventually, I will
%build up to that. But for now, let's just try to build something that
%works. 

%Lets Build the Z vector:

model = BipolarBot();

params = struct('model', model, ...
   'N',200,...
   'nstates', 8, ...
   'ncontrols', 1, ...
   'torque_limit', 6, ...
   'eps', 1e-3);

%Time Constraints:
params.t_init = 0;
params.t_final = 10;
params.t = linspace(params.t_init, params.t_final, params.N+1);
params.t0 = params.t(1);
params.t = params.t(2:end);

desired_pos = [0 0];

z = controlFunction(zeros(8,1),desired_pos,params);


x = reshape(z(1:params.N*params.nstates),params.nstates, params.N)';
%plot(t,x);

model.animate(params.t,x);

