classdef tracking_controller_FP
	properties
        opt_params;
    end

	methods
		function self = tracking_controller_FP(regen, opt_params)
            self.opt_params = opt_params; 
            
            if ~exist(strcat(opt_params.name, '.m'), 'file') || regen
                fprintf('===============================================\n')
                fprintf('Generating forces pro tracking solver: %s\n', strcat(opt_params.name, '.m'))
                generate_forces_pro_tracking_solver(opt_params);
                fprintf('\n')
            end
                
        end

		function [z_pred, u_pred, status, self] = solve(self, z_s, u_prev, z_ref, z_ws, u_ws)
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
                if k == self.opt_params.N+1
                    x0 = [x0; z_ws(:,k)];
                    ub = [ub; z_u];
                    lb = [lb; z_l];
                    params = [params; z_ref(:,k); Q'];
                elseif k == 1
                    x0 = [x0; z_ws(:,k); u_ws(:,k); u_prev];
                    ub = [ub; u_u];
                    lb = [lb; u_l];
                    params = [params; z_ref(:,k); Q'; R'; R_d'];
                else
                    x0 = [x0; z_ws(:,k); u_ws(:,k); u_ws(:,k-1)];
                    ub = [ub; z_u; u_u; u_u];
                    lb = [lb; z_l; u_l; u_l];
                    params = [params; z_ref(:,k); Q'; R'; R_d'];
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
                for k = 1:self.opt_params.N
                    sol = output.(['x',sprintf('%02d', k)]);
                    z_pred(:,k) = sol(1:n_x);
                    u_pred(:,k) = sol(n_x+1:n_x+n_u);
                end
                sol = output.(['x',sprintf('%02d', self.opt_params.N+1)]);
                z_pred(:,self.opt_params.N+1) = sol(1:n_x);
            else
                error('Solver: %s does not exist', strcat(self.opt_params.name, '.m'))
            end
        end
    end
end