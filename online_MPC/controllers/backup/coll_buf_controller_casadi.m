classdef coll_buf_controller_casadi
    properties
        N;
        Q;
        R;
        
        u_u;
        u_l;
        du_u;
        du_l;
        dt;
        
        n_x;
        n_u;
        n_obs;
        n_ineq;
        d_ineq;
        EV_r
        
        opti;
        obj;
        z;
        u;
        z_s;
        u_prev;
        z_ref;
        obs_ph;
        
        dynamics;
    end
    
    methods
        function self = coll_buf_controller_casadi(params)
            % n_ineq: number of inequalities for obstacle description
            % d_ineq: dimension of inequalities for obstacle description
            self.N = params.N;
            self.Q = params.Q;
            self.R = params.R;
            self.dynamics = params.dynamics;
            self.n_obs = params.n_obs;
            self.n_x = params.n_x;
            self.n_u = params.n_u;
            self.u_u = params.u_u;
            self.u_l = params.u_l;
            self.du_u = params.du_u;
            self.du_l = params.du_l;
            self.dt = params.dt;
            self.EV_r = params.EV_r;
            
            import casadi.*
            
            % ========== Setup dynamics ==========
            z_k = MX.sym('z_k', self.n_x);
            u_k = MX.sym('u_k', self.n_u);
            z_kp1 = self.dynamics.f_dt(z_k, u_k);
            f_dyn_dt = Function('dt_dynamics', {z_k, u_k}, {z_kp1}, {'z_k', 'u_k'}, {'z_kp1'});
            
            % ========== Build OBCA optimizer ==========
            self.opti = casadi.Opti();
            
            self.z = self.opti.variable(self.n_x, self.N+1);
            self.u = self.opti.variable(self.n_u, self.N);
            self.z_ref = self.opti.parameter(self.n_x, self.N+1);
            self.z_s = self.opti.parameter(self.n_x);
            self.u_prev = self.opti.parameter(self.n_u);
            self.obs_ph = cell(self.n_obs, self.N);

            self.obj = 0;
            
            % Initial condition
            self.opti.subject_to(self.z(:,1) == self.z_s);
            % Input rate constraint
            self.opti.subject_to(self.du_l*self.dt <= self.u(:,1)-self.u_prev <= self.du_u*self.dt);
            
            for k = 1:self.N
                % Dynamics constraints
                self.opti.subject_to(self.z(:,k+1) == f_dyn_dt(self.z(:,k), self.u(:,k)));
                
                % Input constraints
                self.opti.subject_to(self.u_l <= self.u(:,k) <= self.u_u);
                if k < self.N
                    self.opti.subject_to(self.du_l*self.dt <= self.u(:,k+1)-self.u(:,k) <= self.du_u*self.dt);
                end

                % Obstacle avoidance constraints
                for i = 1:self.n_obs
                    % Set up placeholder variables for obstacle description
                    self.obs_ph{i,k}.pos = self.opti.parameter(2,1);
                    self.obs_ph{i,k}.r = self.opti.parameter(1);

                    self.opti.subject_to(bilin(eye(2), self.z(1:2,k+1)-self.obs_ph{i,k}.pos, self.z(1:2,k+1)-self.obs_ph{i,k}.pos) >= (self.EV_r+self.obs_ph{i,k}.r)^2);
                end
                
                % Tracking and input cost
                self.obj = self.obj + bilin(self.Q, self.z(:,k+1)-self.z_ref(:,k+1), self.z(:,k+1)-self.z_ref(:,k+1)) + ...
                    bilin(self.R, self.u(:,k), self.u(:,k));
            end
            
            self.opti.minimize(self.obj);
            solver_opts = struct('mu_strategy', 'adaptive', ...
                        'mu_init', 1e-5, ...
                        'mu_min', 1e-15, ...
                        'barrier_tol_factor', 1, ...
                        'print_level', 3, ...
                        'max_iter', 300);
            plugin_opts = struct('verbose', 0, 'print_time', 0, 'print_out', 0);
            self.opti.solver('ipopt', plugin_opts, solver_opts);
        end
        
        function [z_pred, u_pred, status, self] = solve(self, z_s, u_prev, z_ref, z_ws, u_ws, obs)
            % Set warm start value
            self.opti.set_initial(self.z, z_ws);
            self.opti.set_initial(self.u, u_ws);
            
            self.opti.set_value(self.z_s, z_s);
            self.opti.set_value(self.u_prev, u_prev);
            self.opti.set_value(self.z_ref, z_ref);
            
            for i = 1:self.n_obs
                for k = 1:self.N
                    self.opti.set_value(self.obs_ph{i,k}.pos, obs{i,k+1}.pos);
                    self.opti.set_value(self.obs_ph{i,k}.r, obs{i,k+1}.r);
                end
            end
            
            try
                tic;
                sol = self.opti.solve();
                solve_time = toc;
            catch
                status = self.opti.stats;
                z_pred = nan*ones(self.n_x, self.N+1);
                u_pred = nan*ones(self.n_u, self.N);
                return;
            end
            
            status = sol.stats;
            status.solve_time = solve_time;
            z_pred = sol.value(self.z);
            u_pred = sol.value(self.u);
        end
    end
end
        