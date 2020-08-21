classdef HppController < MpcController
	methods
		%% HppController: constructor
		function self = HppController(N, EV)
			disp('Constructing HPP controller');
			self@MpcController(N, EV);
		end

		%% solve: solve one iteration
		function [z_opt, u_opt, err] = solve(self, z0, u0, z_ref, hyp)

			tic

			disp('Solving HPP Controller');
			
			% State warm start
			z_WS = z_ref;

			% Initial State
			self.constr = [self.constr, self.z(:, 1) == z0];
			self.constr = [self.constr, -self.u_rate_lim <= self.u(:, 1) - u0 <= self.u_rate_lim];

			for k = 1:self.N+1
				% Hyperplane Constraints
				if ~isempty(hyp(k).w)
					w = hyp(k).w;
					b = hyp(k).b;
					z_WS(1:2, k) = hyp(k).pos;

					self.constr = [self.constr, w(1)*self.z(1, k) + w(2)*self.z(2, k) >= b];
				end

				self.cost = self.cost ...
						+ self.z_coeff(1)*(self.z(1, k) - z_ref(1, k))^2 ...
						+ self.z_coeff(2)*(self.z(2, k) - z_ref(2, k))^2 ...
						+ self.z_coeff(3)*(self.z(3, k) - z_ref(3, k))^2 ...
						+ self.z_coeff(4)*(self.z(4, k) - z_ref(4, k))^2;
			end

			assign(self.z, z_WS);
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