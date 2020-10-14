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
            
            x0 = [];
            params = [];
            for k = 1:self.opt_params.N+1
                params = [params; z_ref(:,k)];
                    
                if k == self.opt_params.N+1
                    x0 = [x0; z_ws(:,k)];
                else
                    x0 = [x0; z_ws(:,k); u_ws(:,k)];
                end
            end
            
            problem.x0 = x0;
            problem.all_parameters = params;
            problem.xinit = z_s;
            
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