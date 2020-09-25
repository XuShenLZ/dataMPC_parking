classdef PID
    properties
        P = 0;
        I = 0;
        D = 0;
        
        x_ref = 0;
        u_ref = 0
        
        u_max = inf;
        u_min = -inf;
        int_max = inf;
        int_min = -inf;
        
        e = 0;
        de = 0;
        ei = 0;
        dt = 0;
        
        x = 0;
    end
    methods
        function obj = PID(P, I, D, dt, x_ref, u_ref, u_lim, int_lim)
            obj.P = P;
            obj.I = I;
            obj.D = D;
            
            if ~isempty(x_ref)
                obj.x_ref = x_ref;
            end
            
            if ~isempty(u_ref)
                obj.u_ref = u_ref;
            end
            
            if ~isempty(u_lim)
                obj.u_max = u_lim(2);
                obj.u_min = u_lim(1);
            end
            
            if ~isempty(int_lim)
                obj.int_max = int_lim(2);
                obj.int_min = int_lim(1);
            end
            
            obj.dt = dt;
        end
        
        function [u, obj] = solve(obj, x)
            obj.x = x;
            
            % Compute errors
            e_k = x - obj.x_ref;
            de_k = (e_k - obj.e)/obj.dt;
            ei_k = obj.ei + e_k*obj.dt;
            
            % Anti-windup
            if ei_k > obj.int_max
                ei_k = obj.int_max;
            end
            if ei_k < obj.int_min
                ei_k = obj.int_min;
            end
            
            % Compute control action terms
            P_val = obj.P*e_k;
            I_val = obj.I*ei_k;
            D_val = obj.D*de_k;
            
            u = -(P_val + I_val + D_val) + obj.u_ref;
            
            % Saturate input
            u = min(u, obj.u_max);
            u = max(u, obj.u_min);
            
            % Update errors
            obj.e = e_k;
            obj.de = de_k;
            obj.ei = ei_k;
        end
        
        function obj = set_x_ref(obj, x_ref)
            obj.x_ref = x_ref;
            obj.ei = 0;
            obj.e = obj.x - x_ref;
        end
        
        function obj = set_u_ref(obj, u_ref)
            obj.u_ref = u_ref;
        end
    end
end
            