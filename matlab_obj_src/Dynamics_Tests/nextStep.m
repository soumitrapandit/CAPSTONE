function x0 = nextStep(delta_t,x0,con,f)
            st = x0;
            k1 = f(st,con);
            k2 = f(st+(delta_t/2)*k1,con);
            k3 = f(st+(delta_t/2)*k2,con);
            k4 = f(st+delta_t*k3,con);
            st = st+(delta_t/6)*(k1+2*k2+2*k3+k4);
            x0 = full(st);
        end