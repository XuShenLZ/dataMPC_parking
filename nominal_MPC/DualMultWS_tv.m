%% DualMultWS: function description
% z_WS: [x, y, psi, v]'
function [mu_opt, lambda_opt] = DualMultWS_tv(t0, N, Obs, EV, z_WS)

	% Number of obstacles
	nOb = size(Obs, 1);
	
	% Number of hyperplanes
	nHp = [];
	for j = 1:nOb
		nHp = [nHp, length(Obs{j, t0+1}.b)];
	end

	L = EV.L;
	G = EV.G;
	g = EV.g;

	offset = EV.offset;

	disp('Solving DualMultWS Model...');

	lambda = sdpvar(sum(nHp), N);
	mu = sdpvar(4*nOb, N);
	d  = sdpvar(nOb, N);

	obj = 0;

	constr = [lambda(:) >= 0];
	constr = [constr, mu(:) >= 0];

	for k = 1:N

		t = [z_WS(1,k) + offset*cos(z_WS(3,k)); z_WS(2,k) + offset*sin(z_WS(3,k))];
		% t = [z_WS(1,k); z_WS(2,k)];
		R = [cos(z_WS(3,k)), -sin(z_WS(3,k)); sin(z_WS(3,k)), cos(z_WS(3,k))];

		for j = 1:nOb
			A = Obs{j, t0+k}.A;
			b = Obs{j, t0+k}.b;

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

	diagnostics = optimize(constr, obj, ops);

	if diagnostics.problem == 0
		disp('Solved');
		mu_opt = value(mu);
		lambda_opt = value(lambda);
	else
		mu_opt = zeros(4, N);
		lambda_opt = zeros(4, N);
		
		yalmiperror(diagnostics.problem)
	end
