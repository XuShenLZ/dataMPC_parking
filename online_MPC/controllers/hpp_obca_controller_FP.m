classdef hpp_obca_controller_FP
	properties
        ws_params;
        opt_params;
        
        lambda_ws;
        mu_ws;
    end

	methods
		% HppControlFP: constructor
		function self = hpp_obca_controller_FP(regen, ws_params, opt_params)
            self.ws_params = ws_params;
            self.opt_params = opt_params;
            
            if ~exist(strcat(ws_params.name, '.m'), 'file') || regen
                generate_forces_pro_ws_solver(ws_params);
            end
            
            if ~exist(strcat(opt_params.name, '.m'), 'file') || regen
                generate_forces_pro_opt_solver(opt_params);
            end
                
        end
        
        function [status, self] = solve_ws(self, z, u, obs)
            
        end

		function [z_pred, u_pred, status, self] = solve(self, z_s, u_prev, z_ref, hyp)
			
		end

    end
end