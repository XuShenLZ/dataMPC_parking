classdef safety_controller_v2
    properties
        EV_curr = [];
        TV_pred = [];
        last_u = zeros(2,1);
        u = zeros(2,1);
        d_lim = [];
        a_lim = [];
        du_lim = [];
        
        dt = 0;
        
        acc_vel_P = 2;
        acc_vel_I = 1;
        acc_vel_D = 0;
        acc_vel_pid_ref = 0;
        acc_vel_int_lim = [];
        acc_vel_PID_controller;
        
        acc_pos_P = 0.2;
        acc_pos_I = 0.1;
        acc_pos_D = 0;
        acc_pos_pid_ref = 0;
        acc_pos_int_lim = [];
        acc_pos_PID_controller;
        
        a_ref = 0;
        
        steer_P = 0.08;
        steer_I = 0.02;
        steer_D = 0;
        steer_pid_ref = 0;
        d_ref = 0;
        steer_int_lim = [];
        steer_PID_controller;
        
        mode = 1; % 1: velocity tracking mode, 2: position tracking mode
        safety_dist;
        
        brake = false;
    end
    
    methods
        function obj = safety_controller_v2(dt, d_lim, a_lim, du_lim)
            obj.dt = dt;
            obj.d_lim = d_lim;
            obj.a_lim = a_lim;
            obj.du_lim = du_lim;
            
            obj.acc_vel_PID_controller = PID(obj.acc_vel_P, obj.acc_vel_I, obj.acc_vel_D, obj.dt, ...
                0, obj.a_ref, obj.a_lim, obj.acc_vel_int_lim);
            
            obj.acc_pos_PID_controller = PID(obj.acc_pos_P, obj.acc_pos_I, obj.acc_pos_D, obj.dt, ...
                0, obj.a_ref, obj.a_lim, obj.acc_pos_int_lim);
            
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
            
            if obj.mode == 1
                obj.acc_vel_PID_controller = obj.acc_vel_PID_controller.set_x_ref(obj.acc_vel_pid_ref);
                [a, obj.acc_vel_PID_controller] = obj.acc_vel_PID_controller.solve(EV_v);
            elseif obj.mode == 2
                obj.acc_pos_PID_controller = obj.acc_pos_PID_controller.set_x_ref(obj.acc_pos_pid_ref-obj.safety_dist);
                [a, obj.acc_pos_PID_controller] = obj.acc_pos_PID_controller.solve(EV_x);
            else
                error('Safety controller mode not recognized')
            end
            
            if EV_v < 0
                [delta, obj.steer_PID_controller] = obj.steer_PID_controller.solve(EV_y - 10*EV_th);
            else
                delta = 0;
            end

            u = [delta; a];

            % Input Rate constraint
            u = min(u, obj.du_lim*obj.dt + last_u);
            u = max(u, -obj.du_lim*obj.dt + last_u);

            obj.u = u;
            
            if obj.mode == 1 && abs(EV_v - obj.acc_vel_pid_ref) <= 0.1
                obj.safety_dist = abs(TV_pred(1,1) - EV_x);
                obj.mode = 2;
            end
        end
        
        function obj = set_pid_gains(obj, acc_P, acc_I, acc_D, steer_P, steer_I, steer_D)
            obj.acc_P = acc_P;
            obj.acc_I = acc_I;
            obj.acc_D = acc_D;
            
            obj.steer_P = steer_P;
            obj.steer_I = steer_I;
            obj.steer_D = steer_D;
        end
        
        
        function obj = set_acc_ref(obj, pos_ref, vel_ref)
            obj.acc_pos_pid_ref = pos_ref;
            obj.acc_vel_pid_ref = vel_ref;
        end
        
        function obj = set_steer_ref(obj, steer_ref)
            obj.steer_pid_ref = steer_ref;
            obj.steer_PID_controller = obj.steer_PID_controller.set_x_ref(steer_ref);
        end
    end
end