%% check_constr: check the collision satisfaction
function [feasible] = check_constr(N, dt, Obs, EV, z, u, mu, lambda)
	
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

	v0 = 2;

	z0 = [EV.startPose; v0];
	zF = [EV.goalPose;  v0];

	% mu and lambda
	dual_constr = [];
	dual_constr = [dual_constr, -min(lambda, [], 'all')];
	dual_constr = [dual_constr, -min(mu, [], 'all')];

	% z1 and zF
	state_constr = [];
	state_constr = [state_constr, max(abs(z(:, 1) - z0))];
	state_constr = [state_constr, max(abs(z(:, N+1) - zF))];

	% Input constraint
	input_constr = [];
	input_constr = [input_constr, max(abs(u(1, :)) - 0.6)];
	input_constr = [input_constr, max(abs(u(2, :)) - 0.5)];

	dyn_constr = [];
	obs_constr = [];
	for k = 1:N

		dyn_constr = [dyn_constr, max(abs(z(:, k+1) - bikeFE(z(:,k), u(:, k), L, dt)))];

		t = [z(1,k) + offset*cos(z(3,k)); z(2,k) + offset*sin(z(3,k))];
		R = [cos(z(3,k)), -sin(z(3,k)); sin(z(3,k)), cos(z(3,k))];
		
		for j = 1:nOb
			A = Obs{j}.A;
			b = Obs{j}.b;

			idx0 = sum( nHp(1:j-1) ) + 1;
			idx1 = sum( nHp(1:j) );
			lambda_j = lambda(idx0:idx1, k);
			mu_j = mu((j-1)*4+1:j*4, k);

			obs_constr = [obs_constr, -(-g'*mu_j + (A*t - b)' * lambda_j - dmin)];

			obs_constr = [obs_constr, max(abs(G'*mu_j + R'*A'*lambda_j))];

			obs_constr = [obs_constr, max(abs(lambda_j'*A*A'*lambda_j-1))];
		end

	end

	feasible = [all(dual_constr < 1e-3), ...
				all(state_constr < 1e-3), ...
				all(input_constr < 1e-3), ...
				all(dyn_constr < 1e-3), ...
				all(obs_constr < 1e-3)];