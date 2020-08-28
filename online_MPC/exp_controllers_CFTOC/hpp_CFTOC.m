%% hpp_CFTOC: path tracking with generated hyperplanes 
function [z_opt, u_opt, feas] = hpp_CFTOC(z0, N, hyp, z_ref, EV)
	% disp('Solving the online controller')

	disp('Online HPP CFTOC');
	tic;

	dt = EV.dt;

	L = EV.L;

	z_WS = z_ref;

	z = sdpvar(4, N+1);
	u = sdpvar(2, N);

	% Initial State
	constr = [];
	constr = [constr, z(:, 1) == z0];

	obj = 0;

	for k = 1:N
		constr = [constr, -0.35 <= u(1, k) <= 0.35];
		constr = [constr, -1 <= u(2, k) <= 1];

		if k < N
			constr = [constr, -0.2 <= u(1, k+1) - u(1, k) <= 0.2];
			constr = [constr, -0.3 <= u(2, k+1) - u(2, k) <= 0.3];
		end

		% constr = [constr, z(:, k+1) == bikeFE(z(:,k), u(:, k), L, dt)];
		constr = [constr, z(:, k+1) == bikeFE_CoG(z(:,k), u(:, k), L, dt)];

		% Hyperplane constraints
		if ~isempty(hyp(k).w)
			w = hyp(k).w;
			b = hyp(k).b;
			z_WS(1:2, k) = hyp(k).pos;

			constr = [constr, w(1)*z(1, k) + w(2)*z(2, k) >= b];

		end

		obj = obj + 0.01*u(1, k)^2 + 0.01*u(2, k)^2 ...
				+ 0.05*(z(1, k) - z_ref(1, k))^2 ...
				+ 0.1*(z(2, k) - z_ref(2, k))^2 ...
				+ 0.1*(z(3, k) - z_ref(3, k))^2 ...
				+ 0.5*(z(4, k) - z_ref(4, k))^2;
	end

	% Terminal Constraint
	if ~isempty(hyp(N+1).w)
		w = hyp(N+1).w;
		b = hyp(N+1).b;
		z_WS(1:2, N+1) = hyp(N+1).pos;

		constr = [constr, w(1)*z(1, N+1) + w(2)*z(2, N+1) >= b];
	end

	obj = obj + 0.05*(z(1, N+1) - z_ref(1, N+1))^2 ...
				+ 0.1*(z(2, N+1) - z_ref(2, N+1))^2 ...
				+ 0.1*(z(3, N+1) - z_ref(3, N+1))^2 ...
				+ 0.5*(z(4, N+1) - z_ref(4, N+1))^2;


	% Assignment
	assign(z, z_WS);
	assign(u, zeros(2, N));


	ops = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 0);

	ops.ipopt.tol = 1e-2;
	ops.ipopt.constr_viol_tol = 1e-3;
	ops.ipopt.max_iter = 300;

	%% Solve
	diagnostics = optimize(constr, obj, ops);

	if diagnostics.problem == 0
		feas = 1;
	else
		feas = 0;
	end

	disp(yalmiperror(diagnostics.problem))

	z_opt = value(z);
	u_opt = value(u);

	toc