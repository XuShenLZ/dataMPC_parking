%% DualMultWS: function description
% z_WS: [x, y, psi, v]'
function [mu_opt, lambda_opt] = DualMultWS(N, As, bs, EV, z_WS)

	nOb = length(As);

	L = EV.L;
	G = EV.G;
	g = EV.g;

	offset = EV.offset;

	lambda = sdpvar(4, N);
	mu = sdpvar(4, N);
	d  = sdpvar(1, N);

	obj = 0;

	constr = [lambda >= 0];
	constr = [constr, mu >= 0];

	for k = 1:N

		t = [z_WS(1,k) + offset*cos(z_WS(3,k)); z_WS(2,k) + offset*sin(z_WS(3,k))];
		R = [cos(z_WS(3,k)), -sin(z_WS(3,k)); sin(z_WS(3,k)), cos(z_WS(3,k))];

		for j = 1:nOb
			A = As{j};
			b = bs{j};
		
			constr = [constr, -g'*mu(:,k) + (A*t - b)' * lambda(:,k) == d(1, k)];

			constr = [constr, G'*mu(:,k) + R'*A'*lambda(:, k) == zeros(2,1)];

			constr = [constr, norm(A'*lambda(:,k)) <= 1];
		end

		obj = obj - d(1, k);
	end

	ops = sdpsettings('solver', 'ipopt', 'verbose', 1);

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
