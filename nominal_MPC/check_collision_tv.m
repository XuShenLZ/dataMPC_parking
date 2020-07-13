%% check_collision_tv: function description
function [collide, goalPose, goal_v] = check_collision_tv(Obs, EV, N, t)
	% ======== Output Initialization

	% Will collide with obstacle
	collide = zeros(1, N);

	% ======= Check the collision between objects

	for k = 1:N

		% Use the prediction of path following MPC
		% to check collision, and determine goalPose
		z = EV.pf_z_opt(:, k+1);
		R = [cos(z(3)), -sin(z(3)); sin(z(3)), cos(z(3))];

		V_ego = (R * EV.V' + z(1:2, 1))';
		P_ego = Polyhedron('V', V_ego);

		% Check the collision between obstacles
		for j = 1:size(Obs, 1)
			inter = P_ego.intersect(Obs{j, t+k});
			if ~inter.isEmptySet
				collide(k) = 1;
				break
			end
		end

	end

	% If collide, scan along the ref path for 
	% safe goal state
	if any(collide)

		safe_ks = find(collide == 0);

		if isempty(safe_ks)
			warning('No safe step found along horizon!');
			% Set safe k = 0
			safe_k = 0;
			% Stop at the current position
			goalPose = EV.pf_z_opt(1:3, 1);
		else
			safe_k = max(safe_ks);
			goalPose = EV.pf_z_opt(1:3, safe_k);
		end


		if safe_k == length(collide)
			goal_v = EV.ref_v;
		else
			goal_v = 0;
		end

	else
		goalPose = EV.pf_z_opt(1:3, end);
		goal_v = EV.ref_v;
	end
