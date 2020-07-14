%% check_collision_tv: function description
function [collide] = check_collision_CFTOC(Obs, EV, traj, T_total)
	% ======== Output Initialization

	% Will collide with obstacle
	collide = false;

	% ======= Check the collision between objects

	for k = 1:T_total

		% Use the prediction of path following MPC
		% to check collision, and determine goalPose
		z = traj(:, k);
		R = [cos(z(3)), -sin(z(3)); sin(z(3)), cos(z(3))];

		V_ego = (R * EV.V' + z(1:2, 1))';
		P_ego = Polyhedron('V', V_ego);

		% Check the collision between obstacles
		for j = 1:size(Obs, 1)
			inter = P_ego.intersect(Obs{j, k});
			if ~inter.isEmptySet
				collide = true;
				return
			end
		end

	end
