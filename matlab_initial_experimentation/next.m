function [t0,x0,u0] = next(T,t0,x0,u,f,params)
st = x0;
con = u(1,:)';
k1 = f(st,con,params);
k2 = f(st+(T/2)*k1,con,params);
k3 = f(st+(T/2)*k2,con,params);
k4 = f(st+T*k3,con,params);
st = st+(T/6)*(k1+2*k2+2*k3+k4);
x0 = full(st);
t0 = t0+T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
end