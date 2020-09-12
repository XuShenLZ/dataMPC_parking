classdef safety_controller
    properties
        EV_curr = [];
        TV_pred = [];
        last_u = zeros(2,1);
        u = zeros(2,1);
        d_lim = [];
        a_lim = [];
        du_lim = [];
        
        dt = 0;
        
        speed_P = 2;
        speed_I = 1;
        speed_D = 0;
        speed_pid_ref = 0;
        a_ref = 0;
        speed_int_lim = [];
        speed_PID_controller;
        
        steer_P = 0.08;
        steer_I = 0.02;
        steer_D = 0;
        steer_pid_ref = 0;
        d_ref = 0;
        steer_int_lim = [];
        steer_PID_controller;
        
        brake = false;
    end
    
    methods
        function obj = safety_controller(dt, d_lim, a_lim, du_lim)
            obj.dt = dt;
            obj.d_lim = d_lim;
            obj.a_lim = a_lim;
            obj.du_lim = du_lim;
            
            obj.speed_PID_controller = PID(obj.speed_P, obj.speed_I, obj.speed_D, obj.dt, ...
                obj.speed_pid_ref, obj.a_ref, obj.a_lim, obj.speed_int_lim);
            
            obj.steer_PID_controller = PID(obj.steer_P, obj.steer_I, obj.steer_D, obj.dt, ...
                obj.steer_pid_ref, obj.d_ref, obj.d_lim, obj.steer_int_lim);
        end
        
        function [u, obj] = solve(obj, EV_curr, TV_pred, last_u)
            obj.last_u = last_u;
            obj.EV_curr = EV_curr;
            obj.TV_pred = TV_pred;

            EV_x = EV_curr(1);
            EV_y = EV_curr(2);
            EV_th = EV_curr(3);
            EV_v = EV_curr(4);
            
            TV_x = TV_pred(1,1);
            TV_y = TV_pred(2,1);
            TV_th = TV_pred(3,1);
            TV_v = TV_pred(4,1);
    
            [a, obj.speed_PID_controller] = obj.speed_PID_controller.solve(EV_v);
            
            if EV_v < 0
                [delta, obj.steer_PID_controller] = obj.steer_PID_controller.solve(EV_y - 10*EV_th);
%                 [delta, obj.steer_PID_controller] = obj.steer_PID_controller.solve(EV_y - EV_th);
            else
                delta = 0;
            end

            u = [delta; a];

            % Input Rate constraint
            u = min(u, obj.du_lim*obj.dt + last_u);
            u = max(u, -obj.du_lim*obj.dt + last_u);

            obj.u = u;
        end
        
        function obj = set_pid_gains(obj, P, I, D)
            obj.P = P;
            obj.I = I;
            obj.D = D;
        end
        
        function obj = set_speed_ref(obj, speed_ref)
            obj.speed_pid_ref = speed_ref;
            obj.speed_PID_controller = obj.speed_PID_controller.set_x_ref(speed_ref);
        end
        
        function obj = set_steer_ref(obj, steer_ref)
            obj.steer_pid_ref = steer_ref;
            obj.steer_PID_controller = obj.steer_PID_controller.set_x_ref(steer_ref);
        end
    end
end