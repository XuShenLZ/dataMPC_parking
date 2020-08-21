classdef NivController < MpcController
	properties
		R
	end

	methods
		%% NivController: constructor
		function self = NivController(N, R, EV)
			disp('Constructing Naive controller');
			self@MpcController(N, EV);
			self.R = R;
		end

		%% solve: solve one iteration
		function [z_opt, u_opt, err] = solve(self, z0, z_ref, TV_pred)

			tic

			disp('Solving Naive Controller');

			% Initial State
			self.constr = [self.constr, self.z(:, 1) == z0];

			for k = 1:self.N+1
				
				% Collision Avoidance Constraints
				self.constr = [self.constr, (self.z(1, k) - TV_pred(1, k))^2 ...
						+ (self.z(2, k) - TV_pred(2, k))^2 >= 4*self.R^2];


				self.cost = self.cost ...
						+ self.z_coeff(1)*(self.z(1, k) - z_ref(1, k))^2 ...
						+ self.z_coeff(2)*(self.z(2, k) - z_ref(2, k))^2 ...
						+ self.z_coeff(3)*(self.z(3, k) - z_ref(3, k))^2 ...
						+ self.z_coeff(4)*(self.z(4, k) - z_ref(4, k))^2;
			end

			assign(self.z, z_ref);
			assign(self.u, zeros(2, self.N));

			%% Solve
			diagnostics = optimize(self.constr, self.cost, self.ops);

			disp(yalmiperror(diagnostics.problem))

			z_opt = value(self.z);
			u_opt = value(self.u);
			err   = diagnostics.problem;
			toc
		end
	end
end