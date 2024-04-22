% This is the main class definition for the BipolarBot.
% @Author - Soumitra Pandit
% @Usage - Initialize and utilize for robot dynamics simulations.
classdef BipolarBot < handle
    properties (Access = public)
        % Physical properties of the robot
        l double = [1; 1];     % Lengths of the links
        m double = [1; 1];     % Masses of the links
        c double = [0.5; 0.5]; % Center of mass positions
        I double = [0.1;0.1];  % Moments of Inertia
    end
    
    properties (Access = public)
        % The dynamics matrices
        M;
        h;
        gr;
       
        dynamicsFunction;
        %state = [x,y,q1,q2,dx,dy,dq1,dq2]
        state = [0;0;0;0;0;0;0;0];
        %dynamics = [dx,dy,dq1,dq2,d2x,d2y,d2q1,d2q2]
        %dynamics = [0;0;0;0;0;0;0;0];
        %control = [0;0;tau1;tau2;alpha]
        control = [0;0;0;0;0];
        g = 9.8;
    end
    
    methods (Access = public)
        % Constructor initializes the robot with default settings.
        function obj = BipolarBot()
            %obj.dynamicsFunction = 
            obj.symbolicDynamics();
        end

        % Initialize symbolic dynamics.
        function symbolicDynamics(obj)
            %-------------------Initialize-------------------------
            syms x y q1 q2 dx dy dq1 dq2 d2x d2y d2q1 d2q2 'real';
            l1 = obj.l(1);
            l2 = obj.l(2);
            m1 = obj.m(1);
            m2 = obj.m(2);
            c1 = obj.c(1);
            c2 = obj.c(2);
            I1 = obj.I(1);
            I2 = obj.I(2);
            %------------------------------------------------------
            %some Organization never hurt anyone
            pos = [x;y];
            vel = [dx;dy];
            %acc = [d2x;d2y]; 
            
            %grouping variables
            q = [x; y; q1; q2];
            dq = [dx; dy; dq1; dq2];
            %d2q = [d2x; d2y; d2q1; d2q2];

            
            %Kinematic Equations:
            p1 = pos + c1*[cos(q1); sin(q1)];
            pj = pos + l1*[cos(q1); sin(q1)];
            p2 = pj + c2*[cos(q1 + q2); sin(q1  + q2)];
            
            %Velocity Kinematics:
            v1 = vel + c1*[-sin(q1); cos(q1)]*dq1;
            v2 = vel + l1*[-sin(q1);cos(q1)]*dq1 + ...
                c2*l2*[-sin(q1+q2);cos(q1+q2)]*(dq1+dq2);

            %Calculate Kinetic Energy
            K = ( m1*(v1'*v1) + m2*(v2'*v2) + I1*dq1^2 + I2*(dq1 + dq2)^2)/2;
            
            %Calculate Potential Energy
            U = m1*obj.g*p1(2) + m2*obj.g*p2(2);

            %Calculate M,C,G
            obj.M = matlabFunction(jacobian(jacobian(K,dq),dq), Vars=[q1, q2]);
            obj.gr = matlabFunction(gradient(U, q), Vars=[q1, q2]);
            obj.h = matlabFunction(jacobian(jacobian(K,dq),q)*dq + gradient(K,q), Vars=[q1, q2, dx, dy, dq1, dq2]);
            
        end

        function dx = dynamics(obj,t,x,u)
            % x = [x y q1 q2 dx dy dq1 dq2];
            % x_dot = [dx dy dq1 dq2 d2x d2y d2q1 d2q2]
            % u = [tau; a];
            
            if(u(2))
                dx(1:2,1) = [0;0];
                dx(3:4,1) = x(7:8,1);
                dx(5:8,1) = obj.M(x(3),x(4))\( [0;0;0;u(1)] - obj.gr(x(3),x(4)) - obj.h(x(3),x(4),0,0,x(7),x(8))  );
            else
                dx(1:4,1) = x(5:8,1);
                dx(5:8,1) = obj.M(x(3),x(4))\( [0;0;0;u(1)] - obj.gr(x(3),x(4)) - obj.h(x(3),x(4),x(5),x(6),x(7),x(8))  );
            end
        end

    end
end
