%% HPPobca_CFTOC: function description
function [z_opt, u_opt, feas] = HPPobca_CFTOC(z0, N, hyp, TV_pred, z_ref, EV)

	disp('Online HPP OBCA');
	tic;

	% ======== Warm Start Using Last Iteration

	[z_WS, u_WS] = extend_prevItr(EV.z_opt, EV.u_opt, EV);

	% =========== Warm Start Using HPP anchor points
	% dt = EV.dt;
	% L = EV.L;
	% % Get z warm start
	% z_WS = z_ref;

	% for k = 1:N+1
	% 	if ~isempty(hyp(k).w)
	% 		z_WS(1:2, k) = hyp(k).pos;
	% 	end
	% end

	% % Compute the u warm start
	% v_WS = z_WS(4, :);

	% delta_WS = atan( diff(z_WS(3,:)) ./ v_WS(1:end-1) * EV.L / dt );

	% a_WS = diff(v_WS) / dt;
	% u_WS = [delta_WS; a_WS];

	% ===========

	% Construct Polyhedra
	for t = 1:N+1
		center_x = TV_pred(1, t);
		center_y = TV_pred(2, t);
		heading = TV_pred(3, t);
		len = EV.length;
		wid = EV.width;

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

		Obs{1, t} = Polyhedron('V', ob_V);
	end

	% Number of obstacles
	nOb = size(Obs, 1);
	
	% Number of hyperplanes
	nHp = [];
	for j = 1:nOb
		nHp = [nHp, length(Obs{j, 1}.b)];
	end

	L = EV.L;
	G = EV.G;
	g = EV.g;
	dt = EV.dt;

	disp('Solving DualMultWS Model...');

	lambda = sdpvar(sum(nHp), N+1);
	mu = sdpvar(4*nOb, N+1);
	d  = sdpvar(nOb, N+1);

	obj = 0;

	constr = [lambda(:) >= 0];
	constr = [constr, mu(:) >= 0];

	for k = 1:N+1

		t = [z_WS(1,k); z_WS(2,k)];
		R = [cos(z_WS(3,k)), -sin(z_WS(3,k)); sin(z_WS(3,k)), cos(z_WS(3,k))];

		for j = 1:nOb
			A = Obs{j, k}.A;
			b = Obs{j, k}.b;

			idx0 = sum( nHp(1:j-1) ) + 1;
			idx1 = sum( nHp(1:j) );
			lambda_j = lambda(idx0:idx1, k);
			mu_j = mu((j-1)*4+1:j*4, k);
		
			constr = [constr, -g'*mu_j + (A*t - b)' * lambda_j == d(j, k)];

			constr = [constr, G'*mu_j + R'*A'*lambda_j == zeros(2,1)];

			constr = [constr, norm(A'*lambda_j) <= 1];

			obj = obj - d(j, k);
		end

	end

	ops = sdpsettings('solver', 'ipopt', 'verbose', 0);

	ops.ipopt.tol = 1e-2;
	ops.ipopt.constr_viol_tol = 1e-3;
	ops.ipopt.max_iter = 300;

	diagnostics = optimize(constr, obj, ops);

	disp(yalmiperror(diagnostics.problem));
	
	mu_WS = value(mu);
	lambda_WS = value(lambda);

	% OBCA
	dmin = 0.001;

	disp('Solving Full Model...');

	z = sdpvar(4, N+1);
	u = sdpvar(2, N);
	lambda = sdpvar(sum(nHp), N+1);
	mu = sdpvar(4*nOb, N+1);

	constr = [lambda(:) >= 0];
	constr = [constr, mu(:) >= 0];

	% Initial State
	constr = [constr, z(:, 1) == z0];

	obj = 0;

	for k = 1:N

		constr = [constr, -0.35 <= u(1, k) <= 0.35];
		constr = [constr, -1 <= u(2, k) <= 1];

		if k < N
			constr = [constr, -0.2 <= u(1, k+1) - u(1, k) <= 0.2];
			constr = [constr, -0.3 <= u(2, k+1) - u(2, k) <= 0.3];
		end

		constr = [constr, z(:, k+1) == bikeFE_CoG(z(:,k), u(:, k), L, dt)];

		obj = obj + 0.01*u(1, k)^2 + 0.01*u(2, k)^2;
	end

	for k = 1:N+1

		t = [z(1,k); z(2,k)];
		R = [cos(z(3,k)), -sin(z(3,k)); sin(z(3,k)), cos(z(3,k))];

		for j = 1:nOb
			A = Obs{j, k}.A;
			b = Obs{j, k}.b;

			idx0 = sum( nHp(1:j-1) ) + 1;
			idx1 = sum( nHp(1:j) );
			lambda_j = lambda(idx0:idx1, k);
			mu_j = mu((j-1)*4+1:j*4, k);

			constr = [constr, -g'*mu_j + (A*t - b)' * lambda_j >= dmin];

			constr = [constr, G'*mu_j + R'*A'*lambda_j == zeros(2,1)];

			constr = [constr, lambda_j'*A*A'*lambda_j == 1];
		end

		% Hyperplane constraints
		if ~isempty(hyp(k).w)
			w = hyp(k).w;
			b = hyp(k).b;

			constr = [constr, w(1)*z(1, k) + w(2)*z(2, k) >= b];
		end

		obj = obj + 0.05*(z(1, k) - z_ref(1, k))^2 ...
				+ 0.1*(z(2, k) - z_ref(2, k))^2 ...
				+ 0.1*(z(3, k) - z_ref(3, k))^2 ...
				+ 0.5*(z(4, k) - z_ref(4, k))^2;
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

	disp(yalmiperror(diagnostics.problem));

	if diagnostics.problem == 0
		feas = 1;
	else
		feas = 0;
	end

	z_opt = value(z);
	u_opt = value(u);
	mu_opt = value(mu);
	lambda_opt = value(lambda);
	toc