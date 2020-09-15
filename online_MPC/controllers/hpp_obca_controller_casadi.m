classdef hpp_obca_controller_casadi
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

        EV_G;
        EV_g;
        
        opti_ws;
        obj_ws;
        lambda_ws;
        mu_ws;
        d_ws;
        z_ws_ph;
        u_ws_ph;
        obs_ws_ph;
        
        opti;
        obj;
        lambda;
        mu;
        d_min;
        z;
        u;
        z_s;
        u_prev;
        z_ref;
        obs_ph;
        hyp_ph;
        
        dynamics;
    end
    
    methods
        function self = hpp_obca_controller_casadi(params)
            % EV_G, EV_g: polytope description of ego vehicle
            % n_ineq: number of inequalities for obstacle description
            % d_ineq: dimension of inequalities for obstacle description
            self.N = params.N;
            self.Q = params.Q;
            self.R = params.R;
            self.dynamics = params.dynamics;
            self.n_obs = params.n_obs;
            self.n_ineq = params.n_ineq;
            self.d_ineq = params.d_ineq;
            self.EV_G = params.G;
            self.EV_g = params.g;
            self.d_min = params.d_min;
            self.n_x = params.n_x;
            self.n_u = params.n_u;
            self.u_u = params.u_u;
            self.u_l = params.u_l;
            self.du_u = params.du_u;
            self.du_l = params.du_l;
            self.dt = params.dt;
            
            import casadi.*
            
            % ========== Build warm start optimizer ==========
            self.opti_ws = casadi.Opti();
            
            % Dual variables for obstacles
            self.lambda_ws = cell(self.n_obs, 1);
            self.mu_ws = cell(self.n_obs, 1);
            self.d_ws = cell(self.n_obs, 1);
            
            for i = 1:self.n_obs
                % Set up warm start dual variables for each obstacle
                self.lambda_ws{i} = self.opti_ws.variable(self.n_ineq, self.N);
                self.mu_ws{i} = self.opti_ws.variable(self.n_ineq, self.N);
                self.d_ws{i} = self.opti_ws.variable(1, self.N);
                
                % (:) vectorizes the variable
                self.opti_ws.subject_to(self.lambda_ws{i}(:) >= 0);
                self.opti_ws.subject_to(self.mu_ws{i}(:) >= 0);
            end

            self.z_ws_ph = self.opti_ws.parameter(self.n_x, self.N+1);
            self.u_ws_ph = self.opti_ws.parameter(self.n_u, self.N);
            self.obs_ws_ph = cell(self.n_obs, self.N);

            self.obj_ws = 0;
            
            % Solves a feasibility problem given the previous trajectory
            % prediction to obtain warm start values for the dual variables
            for k = 1:self.N
                t_ws = self.z_ws_ph(1:2,k+1);
                R_ws = [cos(self.z_ws_ph(3,k+1)), -sin(self.z_ws_ph(3,k+1)); 
                    sin(self.z_ws_ph(3,k+1)), cos(self.z_ws_ph(3,k+1))];
                
                for i = 1:self.n_obs
                    % Set up placeholder variables for obstacle description
                    % (polytopes)
                    self.obs_ws_ph{i,k}.A = self.opti_ws.parameter(self.n_ineq, self.d_ineq);
                    self.obs_ws_ph{i,k}.b = self.opti_ws.parameter(self.n_ineq);

                    self.opti_ws.subject_to(-dot(self.EV_g, self.mu_ws{i}(:,k)) + ...
                        dot(mtimes(self.obs_ws_ph{i,k}.A, t_ws)-self.obs_ws_ph{i,k}.b, self.lambda_ws{i}(:,k)) == self.d_ws{i}(k));
                    self.opti_ws.subject_to(mtimes(self.EV_G', self.mu_ws{i}(:,k)) + ...
                        mtimes(transpose(mtimes(self.obs_ws_ph{i,k}.A, R_ws)), self.lambda_ws{i}(:,k)) == zeros(self.d_ineq, 1));
                    self.opti_ws.subject_to(dot(mtimes(transpose(self.obs_ws_ph{i,k}.A), self.lambda_ws{i}(:,k)), mtimes(transpose(self.obs_ws_ph{i,k}.A), self.lambda_ws{i}(:,k))) <= 1);
                    
                    self.obj_ws = self.obj_ws - self.d_ws{i}(k);
                end
            end
            
            self.opti_ws.minimize(self.obj_ws);
            ws_solver_opts = struct('mu_strategy', 'adaptive', ...
                        'mu_init', 1e-5, ...
                        'mu_min', 1e-15, ...
                        'barrier_tol_factor', 1, ...
                        'print_level', 1, ...
                        'max_iter', 300);
            ws_plugin_opts = struct('verbose', 0, 'print_time', 0, 'print_out', 0);
            self.opti_ws.solver('ipopt', ws_plugin_opts, ws_solver_opts);
            
            % ========== Setup dynamics ==========
            z_k = MX.sym('z_k', self.n_x);
            u_k = MX.sym('u_k', self.n_u);
            z_kp1 = self.dynamics.f_dt(z_k, u_k);
            f_dyn_dt = Function('dt_dynamics', {z_k, u_k}, {z_kp1}, {'z_k', 'u_k'}, {'z_kp1'});
            
            % ========== Build OBCA optimizer ==========
            self.opti = casadi.Opti();
            
            % Dual variables for obstacles
            self.lambda = cell(self.n_obs, 1);
            self.mu = cell(self.n_obs, 1);
            
            for i = 1:self.n_obs
                % Set up dual variables for each obstacle
                self.lambda{i} = self.opti.variable(self.n_ineq, self.N);
                self.mu{i} = self.opti.variable(self.n_ineq, self.N);
                
                % Positivity constraints on dual variables (:) vectorizes the variable
                self.opti.subject_to(self.lambda{i}(:) >= 0);
                self.opti.subject_to(self.mu{i}(:) >= 0);
            end
            
            self.z = self.opti.variable(self.n_x, self.N+1);
            self.u = self.opti.variable(self.n_u, self.N);
            self.z_ref = self.opti.parameter(self.n_x, self.N+1);
            self.z_s = self.opti.parameter(self.n_x);
            self.u_prev = self.opti.parameter(self.n_u);
            self.obs_ph = cell(self.n_obs, self.N);
            self.hyp_ph = cell(1, self.N);

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

                t_opt = self.z(1:2,k+1);
                R_opt = [cos(self.z(3,k+1)), -sin(self.z(3,k+1)); 
                    sin(self.z(3,k+1)), cos(self.z(3,k+1))];
                
                % Obstacle avoidance constraints
                for i = 1:self.n_obs
                    % Set up placeholder variables for obstacle description
                    % (polytopes)
                    self.obs_ph{i,k}.A = self.opti.parameter(self.n_ineq, self.d_ineq);
                    self.obs_ph{i,k}.b = self.opti.parameter(self.n_ineq);

                    self.opti.subject_to(-dot(self.EV_g, self.mu{i}(:,k)) + ...
                        mtimes(transpose(mtimes(self.obs_ph{i,k}.A, t_opt)-self.obs_ph{i,k}.b), self.lambda{i}(:,k)) >= self.d_min);
                    self.opti.subject_to(mtimes(self.EV_G', self.mu{i}(:,k)) + ...
                        mtimes(transpose(mtimes(self.obs_ph{i,k}.A, R_opt)), self.lambda{i}(:,k)) == zeros(self.d_ineq, 1));
                    self.opti.subject_to(dot(mtimes(transpose(self.obs_ph{i,k}.A), self.lambda{i}(:,k)), mtimes(transpose(self.obs_ph{i,k}.A), self.lambda{i}(:,k))) <= 1);
                end
                
                % Hyperplane constraints
                self.hyp_ph{k}.w = self.opti.parameter(self.n_x, 1);
                self.hyp_ph{k}.b = self.opti.parameter(1);
                self.opti.subject_to(dot(self.hyp_ph{k}.w, self.z(:,k+1)) >= self.hyp_ph{k}.b);
                
                % Tracking and input cost
                self.obj = self.obj + bilin(self.Q, self.z(:,k+1)-self.z_ref(:,k+1), self.z(:,k+1)-self.z_ref(:,k+1)) + ...
                    bilin(self.R, self.u(:,k), self.u(:,k));
            end
            
            self.opti.minimize(self.obj);
            solver_opts = struct('mu_strategy', 'adaptive', ...
                        'mu_init', 1e-5, ...
                        'mu_min', 1e-15, ...
                        'barrier_tol_factor', 1, ...
                        'print_level', 1, ...
                        'max_iter', 300);
            plugin_opts = struct('verbose', 0, 'print_time', 0, 'print_out', 0);
            self.opti.solver('ipopt', plugin_opts, solver_opts);
        end
        
        % This function solves for warm start values for the dual variables
        function [status, self] = solve_ws(self, z, u, obs)
            % z, u: predicted state and input sequences (n_x, N+1) and
            % (n_u, N)
            % obs: cell array of size (n_obs, N), each element of cell
            % array should have fields A (n_ineq, d_ineq) and b (n_ineq).
            % This describes the time varying obstacles over the time steps
            % 2:N+1
            self.opti_ws.set_value(self.z_ws_ph, z);
            self.opti_ws.set_value(self.u_ws_ph, u);
            
            for i = 1:self.n_obs
                for k = 1:self.N
                    self.opti_ws.set_value(self.obs_ws_ph{i,k}.A, obs{i,k+1}.A);
                    self.opti_ws.set_value(self.obs_ws_ph{i,k}.b, obs{i,k+1}.b);
                end
            end
            
            try
                tic;
                sol_ws = self.opti_ws.solve();
                solve_time = toc;
            catch
                status = self.opti_ws.stats;
                return;
            end
            
            status = sol_ws.stats;
            status.solve_time = solve_time;
            
            % Set warm start value
            self.opti.set_initial(self.z, z);
            self.opti.set_initial(self.u, u);
            for i = 1:self.n_obs
                self.opti.set_initial(self.lambda{i}, sol_ws.value(self.lambda_ws{i}));
                self.opti.set_initial(self.mu{i}, sol_ws.value(self.mu_ws{i}));
                
                for k = 1:self.N
                    self.opti.set_value(self.obs_ph{i,k}.A, obs{i,k+1}.A);
                    self.opti.set_value(self.obs_ph{i,k}.b, obs{i,k+1}.b);
                end
            end
        end
        
        function [z_pred, u_pred, status, self] = solve(self, z_s, u_prev, z_ref, hyp)
            % hyp: cell array of length N+1 containing the hyperplane
            % constraints for time steps 1:N+1 (self.hyp_ph drops the first time
            % step because of equality constraint)
            self.opti.set_value(self.z_s, z_s);
            self.opti.set_value(self.u_prev, u_prev);
            self.opti.set_value(self.z_ref, z_ref);
            
            for k = 1:self.N
                self.opti.set_value(self.hyp_ph{k}.w, hyp{k+1}.w);
                self.opti.set_value(self.hyp_ph{k}.b, hyp{k+1}.b);
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
        