%% unicycleWS: function description
% Use unicycle model to solve a simplified 
% collision avoidance problem
function [z_WS, feas] = unicycleWS(t0, N, dt, Obs, EV)

	dmin = 1.5 * EV.width;

	% Number of obstacles
	nOb = size(Obs, 1);

	z0 = EV.z0;
	zF = [EV.goalPose; EV.goal_v];

	ref_z = EV.ref_z;

	disp('Solving the simplified unicycle WS model...');

	z = sdpvar(3, N+1); % [x; y; psi]
	u = sdpvar(2, N); % [v; w]

	constr = [];

	constr = [constr, z(:, 1) == z0(1:3, 1)];
	constr = [constr, z(:, N+1) == zF(1:3, 1)];

	obj = 0;

	for k = 1:N

		constr = [constr, z(:, k+1) == unicycle(z(:, k), u(:, k), dt)];

		for j = 1:nOb
			if j <= 2
				% Static obstacles (spots boundary)
				A = Obs{j, t0+k}.A;
				b = Obs{j, t0+k}.b + EV.width*0.75;
 
				constr = [constr, A * z(1:2, k+1) >= b];
			else
				% Dynamic object
				VObs = Obs{j, t0+k}.V;

				center = mean(VObs, 1)';

				constr = [constr, (z(1, k+1) - center(1))^2 ...
								+ (z(2, k+1) - center(2))^2 >= dmin^2];
			end
		end

		obj = obj + 0.01*u(2, k)^2 ... % small w
				+ 0.1*(z(2, k+1) - ref_z(2, k))^2 ... % y close to ref
				+ 0.1*(z(3, k+1) - ref_z(3, k))^2 ... % psi close to ref
				+ 0.05*(u(1, k) - ref_z(4, k))^2; % v close to ref

	end

	assign(z, [z0(1:3, 1), ref_z(1:3, :)]);
	assign(u, [ref_z(4, :); zeros(1, N)]);

	ops = sdpsettings('solver', 'ipopt', 'usex0', 1, 'verbose', 0);

	ops.ipopt.tol = 1e-2;
	ops.ipopt.constr_viol_tol = 1e-3;
	ops.ipopt.max_iter = 300;

	diagnostics = optimize(constr, obj, ops);

	if diagnostics.problem == 0
		disp('Solved');
		feas = 1;
	else
		yalmiperror(diagnostics.problem)
		feas = 0;
	end

	z_opt = value(z);
	u_opt = value(u);

	z_WS = [z_opt;
			u_opt(1, :), u_opt(1, end)];

end

%% unicycle: function description
function zp = unicycle(z, u, dt)
	zp = z;

	zp(1) = z(1) + dt * u(1) * cos(z(3));
	zp(2) = z(2) + dt * u(1) * sin(z(3));
	zp(3) = z(3) + dt * u(2);

end
