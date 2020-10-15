%% emergency_break: Emergency break controller
function [z_opt, u_opt, feas] = emergency_break(T_total, dt, EV, TV)
	[min_x_TV, N] = min(TV.x);

	x_terminal = min_x_TV - TV.length;

	z0 = [EV.z0(1); EV.z0(4)];
	zF = [x_terminal; 0];

	disp('Solving the emergency break...');

	% Double integrator dynamics along x-direction
	A = [1 dt; 0 1];
	B = [0; dt];

	z = sdpvar(2, N+1);
	u = sdpvar(1, N);

	constr = [];

	constr = [constr, z(:, 1) == z0];
	constr = [constr, z(:, N+1) == zF];

	obj = 0;

	for k = 1:N

		% Dynamics
		constr = [constr, z(:, k+1) == A*z(:, k) + B*u(1, k)];

		% Input constraint
		constr = [constr, -2.5 <= u(1, k) <= 1.5];
		if k < N
			constr = [constr, -0.8 <= u(1, k+1) - u(1, k) <= 0.5];
		end

		% Speed constraint
		constr = [constr, 0 <= z(2, k) <= 1.5];

		% Objective
		obj = obj + u(1, k)^2;

	end

	ops = sdpsettings('solver', 'quadprog', 'verbose', 0);

	%% Solve
	feas = 0;
	diagnostics = optimize(constr, obj, ops);

	if diagnostics.problem == 0
		disp('Solved');
		feas = 1;
	else
		yalmiperror(diagnostics.problem)
		feas = 0;
	end

	zz = value(z);
	uu = value(u);

	if N == T_total
		% [x; y; heading; v]
		z_opt = [zz(1, :);
				 EV.ref_y * ones(1, N+1);
				 EV.ref_h * ones(1, N+1);
				 zz(2, :)];

		% [delta; a]
		u_opt = [zeros(1, N);
				 uu];
	else
		z_opt = [zz(1, :), zz(1, end) * ones(1, T_total-N);
				 EV.ref_y * ones(1, T_total+1);
				 EV.ref_h * ones(1, T_total+1);
				 zz(2, :), zeros(1, T_total-N)];

		% [delta; a]
		u_opt = [zeros(1, T_total);
				 uu, zeros(1, T_total-N)];
	end