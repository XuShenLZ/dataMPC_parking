classdef obca_controller_yalmip
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
        
        lambda_ws;
        mu_ws;
        d_ws;
        z_ws;
        u_ws;
        
        lambda;
        mu;
        d_min;
        z;
        u;
        
        dynamics;
    end
    
    methods
        function self = obca_controller_yalmip(params)
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
        end
        
        % This function solves for warm start values for the dual variables
        function [status, self] = solve_ws(self, z_ws, u_ws, obs)
            % z, u: predicted state and input sequences (n_x, N+1) and
            % (n_u, N)
            % obs: cell array of size (n_obs, N), each element of cell
            % array should have fields A (n_ineq, d_ineq) and b (n_ineq).
            % This describes the time varying obstacles over the time steps
            % 2:N+1
            
            % Dual variables for obstacles
            self.lambda_ws = cell(self.n_obs, 1);
            self.mu_ws = cell(self.n_obs, 1);
            self.d_ws = cell(self.n_obs, 1);
            
            self.z_ws = z_ws;
            self.u_ws = u_ws;
            
            ws_objective = 0;
            ws_constraints = [];
            
            for i = 1:self.n_obs
                % Set up dual variables for each obstacle
                self.lambda_ws{i} = sdpvar(self.n_ineq, self.N);
                self.mu_ws{i} = sdpvar(self.n_ineq, self.N);
                self.d_ws{i} = sdpvar(1, self.N);
                
                % Positivity constraints on dual variables (:) vectorizes the variable
                ws_constraints = [ws_constraints, self.lambda_ws{i}(:) >= 0, self.mu_ws{i}(:) >= 0];
            end
            
            for k = 1:self.N
                t_ws = z_ws(1:2,k+1);
                R_ws = [cos(z_ws(3,k+1)), -sin(z_ws(3,k+1)); 
                    sin(z_ws(3,k+1)), cos(z_ws(3,k+1))];
                
                for i = 1:self.n_obs
                    A = obs{i,k+1}.A;
                    b = obs{i,k+1}.b;
                    
                    ws_constraints = [ws_constraints, -self.EV_g'*self.mu_ws{i}(:,k)+(A*t_ws-b)'*self.lambda_ws{i}(:,k) == self.d_ws{i}(k), ...
                        self.EV_G'*self.mu_ws{i}(:,k)+R_ws'*A'*self.lambda_ws{i}(:,k) == zeros(self.d_ineq,1), ...
                        self.lambda_ws{i}(:,k)'*(A*A')*self.lambda_ws{i}(:,k) <= 1];
                    
                    ws_objective = ws_objective - self.d_ws{i}(k);
                end
            end
            
            % Solver options
            opts = sdpsettings('solver', 'ipopt', 'verbose', 1);
            
            tic;
            status = optimize(ws_constraints, ws_objective, opts);
            status.solve_time = toc;

            status.return_status = yalmiperror(status.problem);
            if status.problem == 0
                status.success = true;
            else
                status.success = false;
            end
        end
        
        function [z_pred, u_pred, status, self] = solve(self, z_s, u_prev, z_ref, obs)
            % Dual variables for obstacles
            self.lambda = cell(self.n_obs, 1);
            self.mu = cell(self.n_obs, 1);
            
            objective = 0;
            constraints = [];
            
            for i = 1:self.n_obs
                % Set up dual variables for each obstacle
                self.lambda{i} = sdpvar(self.n_ineq, self.N);
                self.mu{i} = sdpvar(self.n_ineq, self.N);
                
                % Initial guess
                assign(self.lambda{i}, value(self.lambda_ws{i}));
                assign(self.mu{i}, value(self.mu_ws{i}));
                
                % Positivity constraints on dual variables (:) vectorizes the variable
                constraints = [constraints, self.lambda{i}(:) >= 0, self.mu{i}(:) >= 0];
            end
            
            self.z = sdpvar(self.n_x, self.N+1);
            self.u = sdpvar(self.n_u, self.N);
            
            % Initial guess
            assign(self.z, self.z_ws);
            assign(self.u, self.u_ws);
            
            % Initial condition
            constraints = [constraints, self.z(:,1) == z_s];
            % Input rate constraint
            constraints = [constraints, self.dt*self.du_l <= self.u(:,1)-u_prev <= self.dt*self.du_u];
            
            for k = 1:self.N
                % Dynamics constraints
                constraints = [constraints, self.z(:,k+1) == self.dynamics.f_dt(self.z(:,k), self.u(:,k))];
                
                % Input constraints
                constraints = [constraints, self.u_l <= self.u(:,k) <= self.u_u];
                if k < self.N
                    constraints = [constraints, self.dt*self.du_l <= self.u(:,k+1)-self.u(:,k) <= self.dt*self.du_u];
                end
                
                t_opt = self.z(1:2,k+1);
                R_opt = [cos(self.z(3,k+1)), -sin(self.z(3,k+1)); 
                    sin(self.z(3,k+1)), cos(self.z(3,k+1))];
                
                % Obstacle avoidance constraints
                for i = 1:self.n_obs
                    A = obs{i,k+1}.A;
                    b = obs{i,k+1}.b;
                    
                    constraints = [constraints, -self.EV_g'*self.mu{i}(:,k)+(A*t_opt-b)'*self.lambda{i}(:,k) >= self.d_min, ...
                        self.EV_G'*self.mu{i}(:,k)+R_opt'*A'*self.lambda{i}(:,k) == zeros(self.d_ineq,1), ...
                        self.lambda{i}(:,k)'*(A*A')*self.lambda{i}(:,k) <= 1];
                end
                
                % Objective
                objective = objective + (self.z(:,k+1)-z_ref(:,k+1))'*self.Q*(self.z(:,k+1)-z_ref(:,k+1)) + self.u(:,k)'*self.R*self.u(:,k);
                disp(k)
            end
            
            % Solver options
            opts = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 2);
			opts.ipopt.tol = 1e-2;
			opts.ipopt.constr_viol_tol = 1e-3;
			opts.ipopt.max_iter = 300;
            
            tic;
            status = optimize(constraints, objective, opts);
            status.solve_time = toc;

            status.return_status = yalmiperror(status.problem);
            if status.problem == 0
                status.success = true;
            else
                status.success = false;
            end
            
            z_pred = value(self.z);
            u_pred = value(self.u);
        end
    end
end
        