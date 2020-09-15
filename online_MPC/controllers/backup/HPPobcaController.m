classdef HPPobcaController < MpcController
	properties
		G
		g
		dmin = 0.001;
	end

	methods
		%% HPPobcaController: constructor
		function self = HPPobcaController(N, EV)
			disp('Constructing HPPobca controller');
			self@MpcController(N, EV);

			self.G = EV.G;
			self.g = EV.g;
		end

		%% solve: solve one iteration
		function [z_opt, u_opt, err] = solve(self, z0, u0, z_ref, hyp, TV_pred, EV)

			tic

			disp('Solving HPPobca Controller');

			% Construct Polyhedra
			len = EV.length;
			wid = EV.width;
			for k = 1:self.N+1
				center_x = TV_pred(1, k);
				center_y = TV_pred(2, k);
				heading = TV_pred(3, k);

				% Target Vehicle
				Vx = [center_x + len/2*cos(heading) - wid/2*sin(heading);
					  center_x + len/2*cos(heading) + wid/2*sin(heading);
					  center_x - len/2*cos(heading) + wid/2*sin(heading);
					  center_x - len/2*cos(heading) - wid/2*sin(heading)];

				Vy = [center_y + len/2*sin(heading) + wid/2*cos(heading);
					  center_y + len/2*sin(heading) - wid/2*cos(heading);
					  center_y - len/2*sin(heading) - wid/2*cos(heading);
					  center_y - len/2*sin(heading) + wid/2*cos(heading)];

				ob_V = [Vx, Vy];

				Obs{1, k} = Polyhedron('V', ob_V);
				% Obs{2, k} = Polyhedron('A', [0  1], 'b', -self.y_lim);
				% Obs{3, k} = Polyhedron('A', [0 -1], 'b', -self.y_lim);
			end

			% Number of obstacles
			nOb = size(Obs, 1);
			
			% Number of hyperplanes
			nHp = [];
			for j = 1:nOb
				nHp = [nHp, length(Obs{j, 1}.b)];
			end

			% ====================
			disp('Solving DualMultWS Model...');

			% Warm Start
			[z_WS, u_WS] = extend_prevItr(EV.z_opt, EV.u_opt, EV.dt, EV.L);

			lambda = sdpvar(sum(nHp), self.N+1);
			mu = sdpvar(4*nOb, self.N+1);
			d  = sdpvar(nOb, self.N+1);

			obj = 0;

			dual_constr = [lambda(:) >= 0];
			dual_constr = [dual_constr, mu(:) >= 0];

			for k = 1:self.N+1

				t = [z_WS(1,k); z_WS(2,k)];
				R = [cos(z_WS(3,k)), -sin(z_WS(3,k)); sin(z_WS(3,k)), cos(z_WS(3,k))];

				for j = 1:nOb
					A = Obs{j, k}.A;
					b = Obs{j, k}.b;

					idx0 = sum( nHp(1:j-1) ) + 1;
					idx1 = sum( nHp(1:j) );
					lambda_j = lambda(idx0:idx1, k);
					mu_j = mu((j-1)*4+1:j*4, k);
				
					dual_constr = [dual_constr, -self.g'*mu_j + (A*t - b)' * lambda_j == d(j, k)];
					dual_constr = [dual_constr, self.G'*mu_j + R'*A'*lambda_j == zeros(2,1)];
					dual_constr = [dual_constr, norm(A'*lambda_j) <= 1];

					obj = obj - d(j, k);
				end

			end

			ops = sdpsettings('solver', 'ipopt', 'verbose', 0);

			diagnostics = optimize(dual_constr, obj, ops);

			disp(yalmiperror(diagnostics.problem));
	
			mu_WS = value(mu);
			lambda_WS = value(lambda);

			% ====================

			% ====================
			% OBCA

			disp('Solving Full Model...');

			% self.lambda = sdpvar(sum(nHp), self.N+1);
			% self.mu = sdpvar(4*nOb, self.N+1);

			% Constraints on Dual Variables
			self.constr = [self.constr, lambda(:) >= 0];
			self.constr = [self.constr, mu(:) >= 0];

			% Initial State
			self.constr = [self.constr, self.z(:, 1) == z0];
			self.constr = [self.constr, -self.u_rate_lim <= self.u(:, 1) - u0 <= self.u_rate_lim];

			for k = 1:self.N+1

				t = [self.z(1,k); self.z(2,k)];
				R = [cos(self.z(3,k)), -sin(self.z(3,k)); sin(self.z(3,k)), cos(self.z(3,k))];

				for j = 1:nOb
					A = Obs{j, k}.A;
					b = Obs{j, k}.b;

					idx0 = sum( nHp(1:j-1) ) + 1;
					idx1 = sum( nHp(1:j) );
					lambda_j = lambda(idx0:idx1, k);
					mu_j = mu((j-1)*4+1:j*4, k);

					self.constr = [self.constr, -self.g'*mu_j + (A*t - b)' * lambda_j >= self.dmin];

					self.constr = [self.constr, self.G'*mu_j + R'*A'*lambda_j == zeros(2,1)];

					self.constr = [self.constr, lambda_j'*A*A'*lambda_j == 1];
				end

				% Hyperplane constraints
				if ~isempty(hyp(k).w)
					w = hyp(k).w;
					b = hyp(k).b;

					self.constr = [self.constr, w(1)*self.z(1, k) + w(2)*self.z(2, k) >= b];
				end

				self.cost = self.cost ...
						+ self.z_coeff(1)*(self.z(1, k) - z_ref(1, k))^2 ...
						+ self.z_coeff(2)*(self.z(2, k) - z_ref(2, k))^2 ...
						+ self.z_coeff(3)*(self.z(3, k) - z_ref(3, k))^2 ...
						+ self.z_coeff(4)*(self.z(4, k) - z_ref(4, k))^2;

			end

			%% Assignment
			assign(self.z, z_WS);
			assign(self.u, u_WS);
			assign(mu, mu_WS);
			assign(lambda, lambda_WS);

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