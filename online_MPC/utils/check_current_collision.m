%% check_current_collision: function description
function collide = check_current_collision(z_EV, z_TV, EV)
	EV_x  = z_EV(1);
    EV_y  = z_EV(2);
    EV_th = z_EV(3);

	z = [EV_x; EV_y];
	R = [cos(EV_th), -sin(EV_th); sin(EV_th), cos(EV_th)];

	V_ego = (R * EV.V' + z(1:2, 1))';
	P_ego = Polyhedron('V', V_ego);

    TV_x = z_TV(1);
    TV_y = z_TV(2);
    TV_th = z_TV(3);

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