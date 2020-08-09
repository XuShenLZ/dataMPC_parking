%% niv_CFTOC: path tracking with generated hyperplanes 
function [z_opt, u_opt, feas] = niv_CFTOC(z0, N, TV_pred, R, z_ref, EV)
	% disp('Solving the online controller')

	dt = EV.dt;

	L = EV.L;
	offset = EV.offset;

	z = sdpvar(4, N+1);
	u = sdpvar(2, N);

	% Initial State
	constr = [];
	constr = [constr, z(1, 1) == z0(1) - offset*cos(z0(3))];
	constr = [constr, z(2, 1) == z0(2) - offset*sin(z0(3))];
	constr = [constr, z(3:4, 1) == z0(3:4)];

	obj = 0;

	for k = 1:N
		constr = [constr, -0.6 <= u(1, k) <= 0.6];
		constr = [constr, -0.5 <= u(2, k) <= 0.5];

		if k < N
			constr = [constr, -0.2 <= u(1, k+1) - u(1, k) <= 0.2];
			constr = [constr, -0.3 <= u(2, k+1) - u(2, k) <= 0.3];
		end

		constr = [constr, z(:, k+1) == bikeFE(z(:,k), u(:, k), L, dt)];

		constr = [constr, (z(1, k)+offset*cos(z(3, k)) - TV_pred(1, k))^2 ...
						+ (z(2, k)+offset*sin(z(3, k)) - TV_pred(2, k))^2 >= 4*R^2];

		obj = obj + 0.01*u(1, k)^2 + 0.01*u(2, k)^2 ...
				+ 0.5*(z(1, k) - z_ref(1, k))^2 ...
				+ 0.1*(z(2, k) - z_ref(2, k))^2 ...
				+ 0.1*(z(3, k) - z_ref(3, k))^2;
	end

	% Terminal Constraint
	constr = [constr, (z(1, N+1)+offset*cos(z(3, N+1)) - TV_pred(1, N+1))^2 ...
					+ (z(2, N+1)+offset*sin(z(3, N+1)) - TV_pred(2, N+1))^2 >= 4*R^2];

	obj = obj + 0.5*(z(1, N+1) - z_ref(1, N+1))^2 ...
				+ 0.1*(z(2, N+1) - z_ref(2, N+1))^2 ...
				+ 0.1*(z(3, N+1) - z_ref(3, N+1))^2;

	% Assignment
	assign(z, z_ref);
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

	% Transformation
	zz = value(z);

	 for k = 1:N+1
		z_opt(1, k)   = zz(1, k) + offset*cos(zz(3, k));
		z_opt(2, k)   = zz(2, k) + offset*sin(zz(3, k));
		z_opt(3:4, k) = zz(3:4, k);
	end

	u_opt = value(u);