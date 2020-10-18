classdef obca_controller_FP
	properties
        ws_params;
        opt_params;
        
        z_ws;
        u_ws;
        lambda_ws;
        mu_ws;
        
        dual_u;
        dual_l;
    end

	methods
		function self = obca_controller_FP(regen, ws_params, opt_params)
            self.ws_params = ws_params;
            self.opt_params = opt_params;
            
            if ~exist(strcat(ws_params.name, '.m'), 'file') || regen
                fprintf('===============================================\n')
                fprintf('Generating forces pro warm start solver: %s\n', strcat(ws_params.name, '.m'))
                generate_forces_pro_ws_solver(ws_params);
                fprintf('\n')
            end
            
            if ~exist(strcat(opt_params.name, '.m'), 'file') || regen
                fprintf('===============================================\n')
                fprintf('Generating forces pro opt solver: %s\n', strcat(opt_params.name, '.m'))
                generate_forces_pro_opt_solver_naive(opt_params);
                fprintf('\n')
            end
            
            n_obs = opt_params.n_obs;
            n_ineq = opt_params.n_ineq;
            m_ineq = size(opt_params.G,1);
            N_ineq = sum(n_ineq);
            M_ineq = n_obs*m_ineq;
            self.dual_u = inf*ones(N_ineq + M_ineq,1);
            self.dual_l = zeros(N_ineq + M_ineq,1);
        end
        
        function [status, self] = solve_ws(self, z, u, obs)
            n_obs = self.ws_params.n_obs;
            n_ineq = self.ws_params.n_ineq;
            m_ineq = size(self.ws_params.G,1);
            N_ineq = sum(n_ineq);
            M_ineq = n_obs*m_ineq;
            
            self.lambda_ws = nan;
            self.mu_ws = nan;
            self.z_ws = z;
            self.u_ws = u;
            
            % Set up initial guess and parameters
            x0 = [];
            params = [];
            for k = 1:self.ws_params.N+1
                x0 = [x0; zeros(N_ineq+M_ineq+n_obs,1)];
                obs_A = [];
                obs_b = [];
                for i = 1:n_obs
                    obs_A = [obs_A; reshape(obs{i,k}.A, [], 1)];
                    obs_b = [obs_b; reshape(obs{i,k}.b, [], 1)];
                end
                params = [params; z(:,k); obs_A; obs_b];
            end
            
            problem.x0 = x0;
            problem.all_parameters = params;
            
            % Solve
            if exist(strcat(self.ws_params.name, '.m'), 'file')
                [output, exitflag, info] = feval(self.ws_params.name, problem);
                
                if exitflag == 1
                    fprintf('FORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);
                	status.success = true;
                    status.return_status = 'Successfully Solved';
                    status.solve_time = info.solvetime;
                    
                    l_ws = zeros(N_ineq, self.ws_params.N);
                    m_ws = zeros(M_ineq, self.ws_params.N);  
                    for k = 1:self.ws_params.N
                        sol = output.(['x',sprintf('%02d', k)]);
                        j = 0;
                        for i = 1:n_obs
                            l_ws(j+1:j+n_ineq(i),k) = sol(j+1:j+n_ineq(i));
                            m_ws((i-1)*m_ineq+1:i*m_ineq,k) = sol(N_ineq+(i-1)*m_ineq+1:N_ineq+i*m_ineq);
                            j = j + n_ineq(i);
                        end
                    end
                    self.lambda_ws = l_ws;
                    self.mu_ws = m_ws;
                else
                    fprintf('Solving Failed, exitflag = %d\n', exitflag);
                    status.success = false;
                    status.return_status = sprintf('Solving Failed, exitflag = %d', exitflag);
                    status.solve_time = nan;
                end
            else
                error('Solver: %s does not exist', strcat(self.ws_params.name, '.m'))
            end
        end

		function [z_pred, u_pred, status, self] = solve(self, z_s, u_prev, z_ref, obs)
            n_obs = self.opt_params.n_obs;
            n_ineq = self.opt_params.n_ineq;
            m_ineq = size(self.opt_params.G,1);
            N_ineq = sum(n_ineq);
            M_ineq = n_obs*m_ineq;
            
            n_x = self.opt_params.n_x;
            n_u = self.opt_params.n_u;
            
            z_u = self.opt_params.z_u;
            z_l = self.opt_params.z_l;
            u_u = self.opt_params.u_u;
            u_l = self.opt_params.u_l;
            
            Q = self.opt_params.Q;
            R = self.opt_params.R;
            R_d = self.opt_params.R_d;
            
            x0 = [];
            params = [];
            ub = [];
            lb = [];
            for k = 1:self.opt_params.N+1
                obs_A = [];
                obs_b = [];
                for i = 1:n_obs
                    obs_A = [obs_A; reshape(obs{i,k}.A, [], 1)];
                    obs_b = [obs_b; reshape(obs{i,k}.b, [], 1)];
                end
                
%                 params = [params; z_ref(:,k); obs_A; obs_b];
%                 if k == self.opt_params.N+1
%                     x0 = [x0; self.z_ws(:,k); self.lambda_ws(:,self.opt_params.N); self.mu_ws(:,self.opt_params.N)];
%                 else
%                     x0 = [x0; self.z_ws(:,k); self.lambda_ws(:,k); self.mu_ws(:,k); self.u_ws(:,k); self.u_ws(:,k)];
%                 end
                
                if k == self.opt_params.N+1
                    x0 = [x0; self.z_ws(:,k); self.lambda_ws(:,end); self.mu_ws(:,end)];
                    ub = [ub; z_u; self.dual_u];
                    lb = [lb; z_l; self.dual_l];
                    params = [params; z_ref(:,k); obs_A; obs_b; Q'];
                elseif k == 1
                    x0 = [x0; z_s; self.lambda_ws(:,k); self.mu_ws(:,k); self.u_ws(:,k); u_prev];
                    ub = [ub; self.dual_u; u_u];
                    lb = [lb; self.dual_l; u_l];
                    params = [params; z_ref(:,k); obs_A; obs_b; Q'; R'; R_d'];
                else
                    x0 = [x0; self.z_ws(:,k); self.lambda_ws(:,k); self.mu_ws(:,k); self.u_ws(:,k); self.u_ws(:,k-1)];
                    ub = [ub; z_u; self.dual_u; u_u; u_u];
                    lb = [lb; z_l; self.dual_l; u_l; u_l];
                    params = [params; z_ref(:,k); obs_A; obs_b; Q'; R'; R_d'];
                end
            end
            
            problem.x0 = x0;
            problem.all_parameters = params;
            problem.xinit = [z_s; u_prev];
            problem.ub = ub;
            problem.lb = lb;
            
            % Solve
            if exist(strcat(self.opt_params.name, '.m'), 'file')
                [output, exitflag, info] = feval(self.opt_params.name, problem);
                
                if exitflag == 1
                    fprintf('FORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);
                	status.success = true;
                    status.return_status = 'Successfully Solved';
                    status.solve_time = info.solvetime;
                    status.info = info;
                else
                    fprintf('Solving Failed, exitflag = %d\n', exitflag);
                    status.success = false;
                    status.return_status = sprintf('Solving Failed, exitflag = %d', exitflag);
                    status.solve_time = nan;
                    status.info = info;
                end
                
                z_pred = zeros(n_x, self.opt_params.N+1);
                u_pred = zeros(n_u, self.opt_params.N);  
                for k = 1:self.ws_params.N
                    sol = output.(['x',sprintf('%02d', k)]);
                    z_pred(:,k) = sol(1:n_x);
                    u_pred(:,k) = sol(n_x+N_ineq+M_ineq+1:n_x+N_ineq+M_ineq+n_u);
                end
                sol = output.(['x',sprintf('%02d', self.ws_params.N+1)]);
                z_pred(:,self.ws_params.N+1) = sol(1:n_x);
            else
                error('Solver: %s does not exist', strcat(self.ws_params.name, '.m'))
            end
        end
    end
end