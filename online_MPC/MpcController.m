classdef MpcController
	properties
		N
		z
		u
		constr
		cost
		ops
		z_coeff = [0.05 0.1 0.1 0.5];
		u_coeff = [0.01 0.01];
		u_rate_lim = [0.03; 0.3];
	end

	methods
		%% MpcController: function description
		function self = MpcController(N, EV)
			self.N = N;

			dt = EV.dt;
			
			L = EV.L;

			% Decision Var
			self.z = sdpvar(4, N+1);
			self.u = sdpvar(2, N);

			self.constr = [];

			self.cost = 0;

			for k = 1:N
				self.constr = [self.constr, -0.35 <= self.u(1, k) <= 0.35];
				self.constr = [self.constr, -1 <= self.u(2, k) <= 1];

				if k < N
					self.constr = [self.constr, -0.3*dt <= self.u(1, k+1) - self.u(1, k) <= 0.3*dt];
					self.constr = [self.constr, -0.3 <= self.u(2, k+1) - self.u(2, k) <= 0.3];
				end

				self.constr = [self.constr, self.z(:, k+1) == bikeFE_CoG(self.z(:,k), self.u(:, k), L, dt)];

				self.cost = self.cost ...
					+ self.u_coeff(1)*self.u(1, k)^2 + self.u_coeff(2)*self.u(2, k)^2;
			end

			self.ops = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 0);

			self.ops.ipopt.tol = 1e-2;
			self.ops.ipopt.constr_viol_tol = 1e-3;
			self.ops.ipopt.max_iter = 300;
		end
	end
end
		