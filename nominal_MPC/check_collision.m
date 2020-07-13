%% check_collision: function description
% la is the coefficient of look ahead distance
function [collide, deviate, goalPose] = check_collision(Obs, EV, la, N, dt, fig)
	
	% ======== Output Initialization

	% Will collide with obstacle
	collide = false;

	% Has deviated from the ref lane
	deviate = false;

	% ======== Vehicle State and Polyhedron

	% Current vehicle state
	z0 = EV.z0;
	R = [cos(z0(3)), -sin(z0(3)); sin(z0(3)), cos(z0(3))];

	V_ego = (R * EV.V' + z0(1:2, 1))';
	% Ego Polyhedron
	P_ego = Polyhedron('V', V_ego);

	% center of the look ahead polyhedron
	la_center = zeros(2, 1);
	la_center(1) = z0(1) + la * EV.length * cos(z0(3));
	la_center(2) = z0(2) + la * EV.length * sin(z0(3));

	V_la = (R * EV.V' + la_center)';

	% Look ahead polyhedron
	P_la = Polyhedron('V', V_la);

	figure(fig)
	pla_plt = plot(P_la, 'color', 'red', 'alpha', 0.5);
	hold on
	ego_plt = plot(P_ego, 'color', 'blue', 'alpha', 0.5);
	hold on
	pause(0.05)

	% figure

	% Check the collision between obstacles
	for j = 1:length(Obs)
		inter = P_la.intersect(Obs{j});
		if ~inter.isEmptySet
			collide = true;
			break
		end
	end

	% If collide, scan along the ref path for 
	% safe goal state
	if collide

		detect_center = zeros(1, 2);

		detect_x = z0(1);
		while detect_x <= EV.x_max
			detect_center(1,1) = detect_x + la * EV.length;
			detect_center(1,2) = EV.ref_y;

			P_detect = Polyhedron('V', EV.V*2 + detect_center);

			plt_pdetect = plot(P_detect, 'color', 'green', 'alpha', 0.5);
			hold on
			pause(0.1)
			delete(plt_pdetect)

			find_goal = true;
			for j = 1:length(Obs)
				inter = P_detect.intersect(Obs{j});
				if ~inter.isEmptySet
					find_goal = false;
					break
				end
			end

			if find_goal
				goalPose = [detect_center(1);
							EV.ref_y;
							EV.ref_h];
				break
			else
				detect_x = detect_x + dt * EV.ref_v;
			end

		end
	% If not collide, but still away from the lane
	elseif z0(2) > EV.width || z0(2) < -EV.width
		deviate = true;

		goalPose = [z0(1) + (N-1)*dt*EV.ref_v;
					EV.ref_y;
					EV.ref_h];

	else
		deviate = false;
		collide = false;

		goalPose = [z0(1) + (N-1)*dt*EV.ref_v;
					EV.ref_y;
					EV.ref_h];
	end

	delete([pla_plt, ego_plt])