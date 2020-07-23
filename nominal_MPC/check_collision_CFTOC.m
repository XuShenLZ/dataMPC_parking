%% check_collision_tv: check whether there will be a collision
% along the given traj
% ===== Input =====
% Obs: Cell (num_obs, time) MPT Polyhedron of obstacles
% EV: EV struct
% traj: the traj to evaluate
% T_total: Total time steps of traj
% ===== Output =====
% collide: Boolean
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
