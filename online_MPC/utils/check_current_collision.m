%% check_current_collision: function description
function collide = check_current_collision(z_traj, EV, TV, i)
	EV_x  = z_traj(1,i);
    EV_y  = z_traj(2,i);
    EV_th = z_traj(3,i);

	z = [EV_x; EV_y];
	R = [cos(EV_th), -sin(EV_th); sin(EV_th), cos(EV_th)];

	V_ego = (R * EV.V' + z(1:2, 1))';
	P_ego = Polyhedron('V', V_ego);

    TV_x = TV.x(i);
    TV_y = TV.y(i);
    TV_th = TV.heading(i);

    z = [TV_x; TV_y];
	R = [cos(TV_th), -sin(TV_th); sin(TV_th), cos(TV_th)];

	V_target = (R * EV.V' + z(1:2, 1))';
	P_target = Polyhedron('V', V_target);

	inter = P_ego.intersect(P_target);
	if inter.isEmptySet
		collide = false;
	else
		collide = true;
	end