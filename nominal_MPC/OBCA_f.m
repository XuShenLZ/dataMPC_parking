%% OBCA: function description
function [z_opt, u_opt, mu_opt, lambda_opt] = OBCA_f(N, dt, Obs, EV, z_WS, u_WS, mu_WS, lambda_WS)

	dmin = 0.001;

	% Number of obstacles
	nOb = length(Obs);

	% Number of hyperplanes
	nHp = [];
	for j = 1:nOb
		nHp = [nHp, length(Obs{j}.b)];
	end

	L = EV.L;
	G = EV.G;
	g = EV.g;

	offset = EV.offset;

	z0 = EV.z0;
	ref_z = EV.ref_z;

	disp('Constructing Full Model...');

	z = sdpvar(4, N+1);
	u = sdpvar(2, N);
	lambda = sdpvar(sum(nHp), N);
	mu = sdpvar(4*nOb, N);

	constr = [lambda(:) >= 0];
	constr = [constr, mu(:) >= 0];

	constr = [constr, z(:, 1) == z0];

	obj = 0;

	for k = 1:N

		constr = [constr, -0.6 <= u(1, k) <= 0.6];
		constr = [constr, -0.5 <= u(2, k) <= 0.5];

		constr = [constr, z(:, k+1) == bikeFE(z(:,k), u(:, k), L, dt)];

		t = [z(1,k) + offset*cos(z(3,k)); z(2,k) + offset*sin(z(3,k))];
		% t = [z_WS(1,k); z_WS(2,k)];
		R = [cos(z(3,k)), -sin(z(3,k)); sin(z(3,k)), cos(z(3,k))];

		for j = 1:nOb
			A = Obs{j}.A;
			b = Obs{j}.b;

			idx0 = sum( nHp(1:j-1) ) + 1;
			idx1 = sum( nHp(1:j) );
			lambda_j = lambda(idx0:idx1, k);
			mu_j = mu((j-1)*4+1:j*4, k);

			constr = [constr, -g'*mu_j + (A*t - b)' * lambda_j >= dmin];

			constr = [constr, G'*mu_j + R'*A'*lambda_j == zeros(2,1)];

			constr = [constr, lambda_j'*A*A'*lambda_j == 1];
		end

		obj = obj + 0.01*u(1, k)^2 + 0.01*u(2, k)^2 ...
				+ 0.1*(z(2, k+1) - ref_z(2, k))^2 ...
				+ 0.1*(z(3, k+1) - ref_z(3, k))^2 ...
				+ 0.1*(z(4, k+1) - ref_z(4, k))^2;
	end

	%% Assignment
	assign(z, z_WS);
	assign(u, u_WS);
	assign(mu, mu_WS);
	assign(lambda, lambda_WS);

	ops = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 0);
    
    % ops.fmincon.MaxIter = 35;
    % ops.fmincon.OptimalityTolerance = 1e-3;

	ops.ipopt.tol = 1e-2;
	ops.ipopt.constr_viol_tol = 1e-3;
	ops.ipopt.max_iter = 300;
	% ops.ipopt.alpha_for_y = 'min';
	% ops.ipopt.recalc_y = 'yes';
	% ops.ipopt.mumps_mem_percent = 6000;
	% ops.ipopt.min_hessian_perturbation = 1e-12;

	%% Solve
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

	end

	z_opt = value(z);
	u_opt = value(u);
	mu_opt = value(mu);
	lambda_opt = value(lambda);
