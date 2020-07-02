%% check_constr: function description
function [outputs] = check_constr(N, dt, A, b, EV, z, u, mu, lambda)
	L = EV.L;
	G = EV.G;
	g = EV.g;

	offset = EV.offset;

	v0 = 2;

	z0 = [EV.startPose; v0];
	zF = [EV.goalPose;  v0];

	% mu and lambda
	constr = [-min(lambda, [], 'all')];
	constr = [constr, -min(mu, [], 'all')];

	% z1 and zF
	constr = [constr, max(abs(z(:, 1) - z0))];
	constr = [constr, max(abs(z(:, N+1) - zF))];

	% Input constraint
	constr = [constr, -0.6 <= abs(u(1, :)) <= 0.6];

	for k = 1:N

		constr = [constr, -0.6 <= u(1, k) <= 0.6];
		constr = [constr, -0.5 <= u(2, k) <= 0.5];

		constr = [constr, z(:, k+1) == bikeFE(z(:,k), u(:, k), L, dt)];

		t = [z(1,k) + offset*cos(z(3,k)); z(2,k) + offset*sin(z(3,k))];
		R = [cos(z(3,k)), -sin(z(3,k)); sin(z(3,k)), cos(z(3,k))];
		
		constr = [constr, -g'*mu(:,k) + (A*t - b)' * lambda(:,k) >= 0.001];

		constr = [constr, G'*mu(:,k) + R'*A'*lambda(:, k) == zeros(2,1)];

		constr = [constr, lambda(:,k)'*A*A'*lambda(:,k) == 1];

		obj = obj + 0.1*u(1,k)^2 + 0.1*u(2,k)^2 ...
				+ 0.01*(z(1,k) - z_WS(1,k))^2 + 0.01*(z(2,k) - z_WS(2,k))^2 + 0.01*(z(3,k) - z_WS(3,k))^2;
	end
