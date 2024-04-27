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
        M;
        h;
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
            q = sym('q%d',[4,1],'real');
            dq = sym('v%d',[4,1],'real');
            d2q = sym('a%d',[4,1],'real');
            
            [p, dp] = obj.kinematics([q;dq]);
            
            K = obj.m(1)*dp(2,:)*dp(2,:)' + obj.m(2)*dp(4,:)*dp(4,:)' + obj.I(1)*dq(3)^2 + obj.I(2)*(dq(3) + dq(4))^2;
            K = K/2;
            
            U = obj.g*(obj.m(1)*p(2,2) + obj.m(2)*p(4,2));
            
            L = K - U;

            dL_dq = gradient(L,dq);

            eom = jacobian(dL_dq,[q;dq])*[dq;d2q] - gradient(L,q);

            M_ = simplify(jacobian(eom,d2q));
            h_ = simplify(eom - M_*d2q);

            obj.M = sym2fun(M_,{'x'},q,{'x(1)','x(2)','x(3)','x(4)'}, dq, {'x(5)','x(6)','x(7)','x(8)'});
            obj.h = sym2fun(h_,{'x'},q,{'x(1)','x(2)','x(3)','x(4)'}, dq, {'x(5)','x(6)','x(7)','x(8)'}); 
        end

        function [p, dp] = kinematics(obj,x)
            p(1,:) = x(1:2,1)';
            p(2,:) = p(1,:) + obj.c(1)*[cos(x(3)), sin(x(3))];
            p(3,:) = p(1,:) + obj.l(1)*[cos(x(3)), sin(x(3))];
            p(4,:) = p(3,:) + obj.c(2)*[cos(x(3) + x(4)), sin(x(3) + x(4))];
            p(5,:) = p(3,:) + obj.l(2)*[cos(x(3) + x(4)), sin(x(3) + x(4))];
        
            dp(1,:) = x(5:6,1)';
            dp(2,:) = dp(1,:) + obj.c(1)*x(7)*[-sin(x(3)), cos(x(3))];
            dp(3,:) = dp(1,:) + obj.l(1)*x(7)*[-sin(x(3)), cos(x(3))];
            dp(4,:) = dp(3,:) + obj.c(2)*(x(7) + x(8))*[-sin(x(3) + x(4)), cos(x(3) + x(4))];
            dp(5,:) = dp(3,:) + obj.l(2)*(x(7) + x(8))*[-sin(x(3) + x(4)), cos(x(3) + x(4))];
        end

        function dx = dynamics(obj,t,x,u)
            % x = [x y q1 q2 dx dy dq1 dq2];
            % x_dot = [dx dy dq1 dq2 d2x d2y d2q1 d2q2]
            % u = [tau; a];
            
            if(u(2) > 0) % Detached
                dx(1:4,1) = x(5:8,1);
                dx(5:8,1) = obj.M(x)\( [0;0;0;u(1)] - obj.h(x)');
            else    % Attached
                M_ = obj.M(x);
                h_ = obj.h([x(1:4); 0; 0; x(7:8)])';

                dx(1:2,1) = [0;0];
                dx(3:4,1) = x(7:8);
                dx(5:6,1) = [0;0];
                dx(7:8,1) = M_(3:4,3:4)\([0;u(1)] - h_(3:4));
            end

            % if(u(2))
            %     dx(1:2,1) = [0;0];
            %     dx(3:4,1) = x(7:8,1);
            %     dx(5:8,1) = obj.M(x(3),x(4))\( [0;0;0;u(1)] - obj.gr(x(3),x(4)) - obj.h(x(3),x(4),0,0,x(7),x(8))  );
            % else
            %     dx(1:4,1) = x(5:8,1);
            %     dx(5:8,1) = obj.M(x(3),x(4))\( [0;0;0;u(1)] - obj.gr(x(3),x(4)) - obj.h(x(3),x(4),x(5),x(6),x(7),x(8))  );
            % end
        end

    end

    methods(Access=public)
        function animate(obj, t, x)
            % x = [ - x(t = t1) - ; - x(t = t2) - ; - x(t=t3) -]
            % Create figure
            ax = axes(Parent=figure(), NextPlot="add", ...
                DataAspectRatio=[1,1,1],...
                Box="on",...
                Xlim=sum(obj.l)*[-1.2, 1.2],...
                Ylim=sum(obj.l)*[-1.2, 1.2]);
            
            xlabel(ax,'X Position');
            ylabel(ax, 'Y Position');
            title(ax, 'BipolarBot Animation');

            robot = plot(ax,0,0,LineWidth=4,Color=lines(1),LineStyle="-",...
                Marker="o",MarkerFaceColor=lines(1),MarkerSize=10);

            % Animation loop
            %tic
            for k = 1:length(t)
                [p, ~] = kinematics(obj,x(k,:)');
                set(robot, 'XData', p(:,1), 'YData', p(:,2));
                if k>1
                    pause(t(k) - t(k-1)); % Pause to match the simulation time step
                end
            end
        end
    end
end
