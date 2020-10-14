classdef HppControlFP
	properties
		name = 'FP_HPPSolver';
		model = {};
		bike_dyn

		u_lim = [0.35, 1];
		u_rate_lim = [0.3, 3];
		y_lim = 3.5;
	end

	properties (Constant)
		L_r = 1.4714;
		L_f = 1.4714;
		M = 10; % RK4 steps
		dt = 0.1;

		z_coeff = [0.05, 0.1, 0.1, 0.5];
		u_coeff = [0.01, 0.01];
	end

	methods
		%% HppControlFP: constructor
		function self = HppControlFP(N)

			disp('Constructing HPP Controller using Foces Pro');

			self.model.xinitidx = [1:4, 7:8];
			% self.model.xinit = zeros(6, 1);

			self.model.N = N + 1;

			% ref_z has 4 elements, hyperplane has 3 elements
			self.model.npar = 4 + 3;

			self.model.eq = @HppControlFP.dynamics;

			for k = 1:self.model.N

				if k <= self.model.N-1
					% z: [x, y, heading, v, delta, a, delta_prev, a_prev]
					self.model.nvar(k) = 8;
					
					% ineq: hpp and rate constraints
					self.model.nh(k) = 3;

					self.model.ineq{k} = @HppControlFP.ineq;
					self.model.hu{k} = [inf, self.u_rate_lim*self.dt];
					self.model.hl{k} = [0, -self.u_rate_lim*self.dt];

					self.model.ub{k} = [inf, self.y_lim, inf, inf, ...
										self.u_lim, ...
										inf, inf];
					self.model.lb{k} = -self.model.ub{k};

					% eq: dynamics for (x, y, heading, v, delta_prev, a_prev)
					self.model.neq(k) = 6;

					if k < self.model.N - 1
						self.model.E{k} = [1 0 0 0 0 0 0 0;
										   0 1 0 0 0 0 0 0;
										   0 0 1 0 0 0 0 0;
										   0 0 0 1 0 0 0 0;
										   0 0 0 0 0 0 1 0;
										   0 0 0 0 0 0 0 1];
					else
						self.model.E{k} = [eye(4); zeros(2, 4)];
					end

					self.model.objective{k} = @HppControlFP.objective;
				else
					% z: [x, y, heading, v]
					self.model.nvar(k) = 4;

					% ineq: hpp
					self.model.nh(k) = 1;

					self.model.ineq{k} = @HppControlFP.ineqN;
					self.model.hu{k} = inf;
					self.model.hl{k} = 0;

					self.model.ub{k} = [inf, self.y_lim, inf, inf];

					self.model.lb{k} = -self.model.ub{k};

					self.model.objective{k} = @HppControlFP.objectiveN;
				end

			end

		end

		%% gen_solver: generate solver
		function gen_solver(self)
			codeoptions = getOptions(self.name);
			codeoptions.maxit = 300;    % Maximum number of iterations
			codeoptions.printlevel = 2; % Use printlevel = 2 to print progress (but not for timings)
			codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

			codeoptions.nlp.TolStat = 1e-2;
			codeoptions.nlp.TolEq = 1e-3;
			codeoptions.nlp.TolIneq = 1e-3;

			codeoptions.nlp.ad_tool = 'casadi-351';
			codeoptions.BuildSimulinkBlock = 0;
		
			FORCES_NLP(self.model, codeoptions);
		end

		%% solve: solve the problem
		function [z_opt, u_opt, err] = solve(self, z0, u0, z_ref, hyp)
			
			z_WS = z_ref;

			hyp_par = zeros(3, self.model.N);
			for k = 1:self.model.N
				% Hyperplane Constraints
				if ~isempty(hyp(k).w)
					w = hyp(k).w;
					b = hyp(k).b;
					z_WS(1:2, k) = hyp(k).pos;

					hyp_par(:, k) = [w(1); w(2); b];
				else
					hyp_par(:, k) = [1; 1; -1e3];
				end
			end

			% Firstly set N+1 initial guessess for [delta; a; delta_prev; a_prev]
			var_WS = [z_WS; zeros(4, self.model.N)];
			var_WS = reshape(var_WS, [], 1);
			% Remove the last 4 inputs for terminal step
			var_WS = var_WS(1:end-4);

			problem.xinit = [z0; u0];
			problem.x0 = var_WS;
			problem.all_parameters = reshape([z_ref; hyp_par], self.model.npar*self.model.N, 1);

			z_opt = zeros(4, self.model.N);
			u_opt = zeros(2, self.model.N-1);

			if exist([self.name, '.m'])
				[output, exitflag, info] = feval(self.name, problem);

				if exitflag == 1
					fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);

					for k = 1:self.model.N
						TEMP = output.(['x',sprintf('%02d', k)]);
						z_opt(:, k) = TEMP(1:4, 1);
						if k < self.model.N
							u_opt(:, k) = TEMP(5:6, 1);
						end
					end

					err = 'Successfully Solved';

				else
					err = sprintf('Solving Failed, exitflag = %d', exitflag);
				end
			else
				err = 'Solver Not Defined';
			end

			disp(err);
		end

	end

	methods (Static)
		%% dynamics: dynamic model
		function zp1 = dynamics(z)
			L_r = HppControlFP.L_r;
			L_f = HppControlFP.L_f;
			dt = HppControlFP.dt;
			M = HppControlFP.M;

			bike_dyn = bike_dynamics_rk4(L_r, L_f, dt, M);

			x = z(1:4);
			u = z(5:6);

			% xp1 = bike_dyn.f_dt(x, u);
			% Seems faster to use the RK4
			xp1 = RK4(x, u, @bike_dyn.f_ct, dt);

			zp1 = [xp1; u];
		end

		%% ineq: ineq constraints at normal stages
		function h = ineq(z, p)
			x = z(1:4);
			u = z(5:6);
			u_prev = z(7:8);

			hyp = p(5:7);

			h = [hyp(1)*x(1) + hyp(2)*x(2) - hyp(3); ...
				 u(1) - u_prev(1); ...
				 u(2) - u_prev(2)];

		end

		%% ineqN: ineq constraints at terninal stage
		function h = ineqN(z, p)
			x = z(1:4);

			hyp = p(5:7);

			h = hyp(1)*x(1) + hyp(2)*x(2) - hyp(3);
		end

		%% objective: stage objective
		function obj = objective(z, p)
			x = z(1:4);
			u = z(5:6);

			z_ref = p(1:4);

			obj = HppControlFP.u_coeff(1)*u(1)^2 + HppControlFP.u_coeff(2)*u(2)^2;

			for i = 1:4
				obj = obj + HppControlFP.z_coeff(i) * (x(i) - z_ref(i))^2;
			end

		end

		%% objectiveN: ternimal objective
		function obj = objectiveN(z, p)
			x = z(1:4);

			z_ref = p(1:4);

			obj = 0;

			for i = 1:4
				obj = obj + HppControlFP.z_coeff(i) * (x(i) - z_ref(i))^2;
			end
		end
	end

end