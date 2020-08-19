classdef safety_controller
    properties
        EV_curr = [];
        TV_pred = [];
        last_u = zeros(2,1);
        u = zeros(2,1);
        d_lim = [];
        a_lim = [];
        
        dt = 0;
        
        speed_P = 2;
        speed_I = 1;
        speed_D = 0;
        v_ref = -0.5;
        a_ref = 0;
        speed_int_lim = [];
        speed_PID_controller;
        
        steer_P = 1;
        steer_I = 1;
        steer_D = 0;
        y_ref = 0;
        d_ref = 0;
        steer_int_lim = [];
        steer_PID_controller;
    end
    
    methods
        function obj = safety_controller(dt, d_lim, a_lim)
            obj.dt = dt;
            obj.d_lim = d_lim;
            obj.a_lim = a_lim;
            
            obj.speed_PID_controller = PID(obj.speed_P, obj.speed_I, obj.speed_D, obj.dt, ...
                obj.v_ref, obj.a_ref, obj.a_lim, obj.speed_int_lim);
            
            obj.steer_PID_controller = PID(obj.steer_P, obj.steer_I, obj.steer_D, obj.dt, ...
                obj.y_ref, obj.d_ref, obj.d_lim, obj.steer_int_lim);
        end
        
        function [u, obj] = solve(obj, EV_curr, TV_pred, last_u)
            obj.last_u = last_u;
            
            obj.EV_curr = EV_curr;
            obj.TV_pred = TV_pred;
            
            EV_x = EV_curr(1);
            EV_y = EV_curr(2);
            EV_th = EV_curr(3);
            EV_v = EV_curr(4);
            
            TV_x = TV_pred(1,:);
            TV_y = TV_pred(2,:);
            TV_th = TV_pred(3,:);
            TV_v = TV_pred(4,:);
            
            [a, obj.speed_PID_controller] = obj.speed_PID_controller.solve(EV_v);
            
            if EV_v < 0
                [delta, obj.steer_PID_controller] = obj.steer_PID_controller.solve(EV_y - 1.5*EV_th);
            else
                delta = 0;
            end
            
            u = [delta; a];
            obj.u = u;
        end
        
        function obj = set_pid_gains(obj, P, I, D)
            obj.P = P;
            obj.I = I;
            obj.D = D;
        end
    end
end