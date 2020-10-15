%% speed_controller: Try to solve an optimal speed profile
function [z_opt, u_opt, feas] = speed_controller(N, dt, Obs, EV)
	
	dmin = EV.length;

	z0 = EV.z0;
	zF = [EV.goalPose; EV.goal_v];

	ref_z = EV.ref_z;

	disp('Solving the collision free speed profile...');

	% Double integrator dynamics along x-direction
	A = [1 dt; 0 1];
	B = [0; dt];

	z = sdpvar(2, N+1); % [x, v]
	u = sdpvar(1, N); % a

	constr = [];

	constr = [constr, z(1, 1) == z0(1,1)];
	constr = [constr, z(2, 1) == z0(4,1)];
	% constr = [constr, z(1, N+1) == zF(1, 1)];

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

		% Obstacle avoidance constraint
		VObs = Obs{3, k}.V; % dynamic obstacle is #3
		VObs_x = mean(VObs(:, 1));
		VObs_y = mean(VObs(:, 2));

		constr = [constr, (z(1, k) - VObs_x)^2 + ...
							(ref_z(2, k) - VObs_y)^2 >= dmin^2];

		% Objective
		obj = obj + 0.01*u(1, k)^2 ...
				+ 0.1*(z(2, k+1) - ref_z(4, k))^2;

	end

	ops = sdpsettings('solver', 'ipopt', 'verbose', 0);

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

	% [x; y; heading; v]
	z_opt = [zz(1, :);
			z0(2:3, 1), ref_z(2:3, :);
			zz(2, :)];

	% [delta; a]
	u_opt = [zeros(1, N);
			 uu];