%% track_controller: function description
function [z_opt, u_opt, feas] = track_controller(N, dt, EV)
	L = EV.L;

	offset = EV.offset;

	z0 = EV.z0;
	ref_z = EV.ref_z;

	disp('Simple Path Tracking...');

	z = sdpvar(4, N+1);
	u = sdpvar(2, N);

	constr = [];

	% Initial State
	constr = [constr, z(1, 1) == z0(1) - offset*cos(z0(3))];
	constr = [constr, z(2, 1) == z0(2) - offset*sin(z0(3))];
	constr = [constr, z(3:4, 1) == z0(3:4)];

	obj = 0;

	for k = 1:N
		% Input range
		constr = [constr, -0.6 <= u(1, k) <= 0.6];
		constr = [constr, -0.5 <= u(2, k) <= 0.5];

		% Dynamics
		constr = [constr, z(:, k+1) == bikeFE(z(:,k), u(:, k), L, dt)];

		obj = obj + 0.01*u(1, k)^2 + 0.01*u(2, k)^2 ...
				+ 0.1*(z(2, k+1) - ref_z(2, k))^2 ...
				+ 0.1*(z(3, k+1) - ref_z(3, k))^2 ...
				+ 0.1*(z(4, k+1) - ref_z(4, k))^2;

	end

	z_WS = [z0, ref_z];
	u_WS = zeros(2, N);

	assign(z, z_WS);
	assign(u, u_WS);

	ops = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 0);

	ops.ipopt.tol = 1e-2;
	ops.ipopt.constr_viol_tol = 1e-3;
	ops.ipopt.max_iter = 300;

	%% Solve
	diagnostics = optimize(constr, obj, ops);

	if diagnostics.problem == 0
		disp('Solved');
		feas = 1;
	else
		yalmiperror(diagnostics.problem)
		feas = 0;
	end

	zz = value(z);

	for k = 1:N+1
		z_opt(1, k)   = zz(1, k) + offset*cos(zz(3, k));
		z_opt(2, k)   = zz(2, k) + offset*sin(zz(3, k));
		z_opt(3:4, k) = zz(3:4, k);
	end

	u_opt = value(u);