%% OBCA: function description
function [z_opt, u_opt] = OBCA(N, dt, As, bs, EV, z_WS, u_WS, mu_WS, lambda_WS)

	dmin = 0.001;

	nOb = length(As);

	L = EV.L;
	G = EV.G;
	g = EV.g;

	offset = EV.offset;

	v0 = 2;

	z0 = [EV.startPose; v0];
	zF = [EV.goalPose;  v0];

	disp('Constructing Model...');

	z = sdpvar(4, N+1);
	u = sdpvar(2, N);
	lambda = sdpvar(4, N);
	mu = sdpvar(4, N);

	constr = [lambda >= 0];
	constr = [constr, mu >= 0];

	obj = 0;

	for k = 1:N

		if k == 1
			constr = [constr, z(:, 1) == z0];
		elseif k == N
			constr = [constr, z(:, k+1) == zF];
		end

		constr = [constr, -0.6 <= u(1, k) <= 0.6];
		constr = [constr, -0.5 <= u(2, k) <= 0.5];

		constr = [constr, z(:, k+1) == bikeFE(z(:,k), u(:, k), L, dt)];

		t = [z(1,k) + offset*cos(z(3,k)); z(2,k) + offset*sin(z(3,k))];
		R = [cos(z(3,k)), -sin(z(3,k)); sin(z(3,k)), cos(z(3,k))];

		for j = 1:nOb
			A = As{j};
			b = bs{j};

			constr = [constr, -g'*mu(:,k) + (A*t - b)' * lambda(:,k) >= dmin];

			constr = [constr, G'*mu(:,k) + R'*A'*lambda(:, k) == zeros(2,1)];

			constr = [constr, lambda(:,k)'*A*A'*lambda(:,k) == 1];
		end

		obj = obj + 0.1*u(1,k)^2 + 0.1*u(2,k)^2 ...
				+ 0.01*(z(1,k) - z_WS(1,k))^2 + 0.01*(z(2,k) - z_WS(2,k))^2 + 0.01*(z(3,k) - z_WS(3,k))^2;
	end

	%% Assignment
	assign(z, z_WS);
	assign(u, u_WS);
	assign(mu, mu_WS);
	assign(lambda, lambda_WS);

	ops = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 1);

	ops.ipopt.tol = 1e-4;
	ops.ipopt.max_iter = 300;
	ops.ipopt.alpha_for_y = 'min';
	ops.ipopt.recalc_y = 'yes';
	ops.ipopt.mumps_mem_percent = 6000;
	ops.ipopt.min_hessian_perturbation = 1e-12;

	%% Solve
	disp('Start Solving...');
	diagnostics = optimize(constr, obj, ops);

	if diagnostics.problem == 0
		disp('Solved');
	else
		yalmiperror(diagnostics.problem)

		disp('Re-trying...');

		assign(z, value(z));
		assign(u, value(u));
		assign(mu, value(mu));
		assign(lambda, value(lambda));

		diagnostics = optimize(constr, obj, ops);

		if diagnostics.problem == 0
			disp('Solved');
		else
			yalmiperror(diagnostics.problem)
		end

		% z_opt = zeros(4, N+1);
		% u_opt = zeros(2, N);
	end

	z_opt = value(z);
	u_opt = value(u);
