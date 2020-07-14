close('all');
clear('all');
clc

%% Load data
load('traj_data.mat');
map_offset = [210, 285];

%% Experiment number 
exp_num = 13;
close('all');

%% Extract TV data
TV.dims   = ego_dims{exp_num};
TV.width  = TV.dims.width;
TV.length = TV.dims.length;

TV.traj    = ego_trajectory{exp_num};
TV.t       = TV.traj(:, 1);
% Flip x and y
TV.x       = TV.traj(:, 3) - map_offset(1);
TV.y       = TV.traj(:, 2) - map_offset(2);
% Flip x and y -> fix heading
TV.heading = -TV.traj(:, 4) + pi/2;

TV.v       = TV.traj(:, 5);
TV.yawDot  = TV.traj(:, 6);

%% Plot parking lot
parking_lines = squeeze(parking_lot(exp_num, :, :));
% Flip x and y
line_x = parking_lines(:, 2) - map_offset(1);
line_y = parking_lines(:, 1) - map_offset(2);
line_w = parking_lines(:, 4);
line_h = parking_lines(:, 3);
parking_lines = [line_x, line_y, line_w, line_h];

static_objs = squeeze(static_object_list(exp_num, :, :));
% Flip x and y
obj_x = static_objs(:, 2) - map_offset(1);
obj_y = static_objs(:, 1) - map_offset(2);
obj_w = static_objs(:, 4);
obj_h = static_objs(:, 3);
static_objs = [obj_x, obj_y, obj_w, obj_h];

fig_pl = figure;
for i = 1:size(parking_lines, 1)
	parking_line = parking_lines(i, :);
	parking_line(1) = parking_line(1) - parking_line(3)/2;
	parking_line(2) = parking_line(2) - parking_line(4)/2;
	rectangle('Position', parking_line, 'FaceColor', 'k')
	hold on
end

for i = 1:size(static_objs, 1)
	static_obj = static_objs(i, :);
	if static_obj(2) < -10 || static_obj(2) > 10
		continue
	end
	static_obj(1) = static_obj(1) - static_obj(3)/2;
	static_obj(2) = static_obj(2) - static_obj(4)/2;
	rectangle('Position', static_obj, 'FaceColor', [211,211,211]/255, 'EdgeColor', 'none')
	hold on
end
axis equal
% x_l, x_u, y_l, y_u
map_dim = [-30 30 -10 10];
axis(map_dim);
% First dimension (y) and second dimension (x)
map_size = [map_dim(4)-map_dim(3), map_dim(2)-map_dim(1)];

%% Obstacles Definition
T_total = length(TV.t);

for t = 1:T_total
	% Static Obstacles
	Obs{1, t} = Polyhedron('A', [0  1], 'b', -3.5);
	Obs{2, t} = Polyhedron('A', [0 -1], 'b', -3.5);

	center_x = TV.x(t);
	center_y = TV.y(t);
	heading = TV.heading(t);
	len = TV.length;
	wid = TV.width;

	% Target Vehicle
	Vx = [center_x + len/2*cos(heading) - wid/2*sin(heading);
		  center_x + len/2*cos(heading) + wid/2*sin(heading);
		  center_x - len/2*cos(heading) + wid/2*sin(heading);
		  center_x - len/2*cos(heading) - wid/2*sin(heading)];

	Vy = [center_y + len/2*sin(heading) + wid/2*cos(heading);
		  center_y + len/2*sin(heading) - wid/2*cos(heading);
		  center_y - len/2*sin(heading) - wid/2*cos(heading);
		  center_y - len/2*sin(heading) + wid/2*cos(heading)];

	ob_V = [Vx, Vy];

	Obs{3, t} = Polyhedron('V', ob_V);
end

%% Ego Vehicle Attributes and Reference Trajectory

dt = TV.t(2) - TV.t(1); % Should be 0.1s

EV.x_min = map_dim(1);
EV.x_max = map_dim(2);

EV.a_max = 0.5;
EV.delta_max = 0.6;

EV.ref_v = 2; % 2m/s = 7.2 km/h
EV.ref_y = 0;
EV.ref_h = 0;

EV.width = TV.width;
EV.length = TV.length;
EV.L = 0.6 * EV.length;
EV.offset = EV.L / 2;
EV.G = [1 0; -1 0; 0 1; 0 -1];
EV.g = [EV.length/2; EV.length/2; EV.width/2; EV.width/2];
P_EV = Polyhedron('A', EV.G, 'b', EV.g);
EV.V = P_EV.V;

% Decide the starting point
for x0 = [-15, -10, -5, 0, 5, 10]

	EV.z0 = [x0; EV.ref_y; EV.ref_h; EV.ref_v];
	EV.goalPose = [EV.z0(1) + (T_total-1)*dt*EV.ref_v; 
				EV.ref_y; 
				EV.ref_h; 
				EV.ref_v];
	EV.goal_v = EV.ref_v;

	EV.ref_z = [linspace(EV.z0(1), EV.goalPose(1), T_total);
				EV.ref_y * ones(1, T_total);
				EV.ref_h * ones(1, T_total);
				EV.ref_v * ones(1, T_total)];

	% Make sure that there is collision along the way
	collide_ref = check_collision_CFTOC(Obs, EV, EV.ref_z, T_total);
	if collide_ref
		fprintf('Starting point x0 = %.2f \n', x0);
		break
	end

end

%% Plot cars
plt_ops.color = 'blue';
plt_ops.alpha = 0.5;
plt_ops.circle = false;

plt_TV = plot(TV.x, TV.y, 'k--');
plt_EV_ref = plot(EV.ref_z(1, :), EV.ref_z(2, :), 'g.');
for t = 1:T_total

	% Stop plotting early when drive out of frame
	if EV.ref_z(1, t) > EV.x_max
		break
	end

	[pEV, cEV] = plotCar(EV.ref_z(1, t), EV.ref_z(2, t), EV.ref_z(3, t), EV.width, EV.length, plt_ops);
	hold on

	plt_ob1 = plot(Obs{1, t}, 'color', 'black');
	hold on
	plt_ob2 = plot(Obs{2, t}, 'color', 'black');
	hold on
	plt_ob3 = plot(Obs{3, t}, 'color', 'black');
	hold on
	axis(map_dim);
	pause(0.05)

	delete([pEV, cEV])

	delete(plt_ob1)
	delete(plt_ob2)
	delete(plt_ob3)
end

%% Solve for aggressive collision avoidance
try
    error('manual switch')
    
	[z_WS, feas] = unicycleWS(0, T_total, dt, Obs, EV);

	if ~feas
		error('Unicycle WS Infeasible');
	end

	v_WS = z_WS(4, :);

	delta_WS = atan( diff(z_WS(3,:)) ./ v_WS(1:end-1) * EV.L / dt );

	a_WS = diff(v_WS) / dt;
	u_WS = [delta_WS; a_WS];

	% Warm start Dual Mult
	[mu_WS, lambda_WS] = DualMultWS_tv(0, T_total, Obs, EV, z_WS);

	% Solve for collision free traj
	[z_opt, u_opt, mu_opt, lambda_opt, feas] = OBCA_tv(0, T_total, dt, Obs, EV, z_WS, u_WS, mu_WS, lambda_WS);

	if ~feas
		error('Full Opti Infeasible');
	end

catch ME

	disp(ME.message);
	disp('Switched to speed control on ref path...');
	% Solve an optimal speed controller
	[z_opt, u_opt, feas] = speed_controller(T_total, dt, Obs, EV);

	if ~feas
		[z_opt, u_opt, feas] = emergency_break(T_total, dt, EV, TV);

		collide_eb = check_collision_CFTOC(Obs, EV, z_opt, T_total);

		fprintf('Collision of Emergency Break: %d \n', collide_eb)
	end

end

EV.traj = z_opt;
EV.inputs = u_opt;

% %% Plot WS solution
% plt_EV_WS = plot(z_WS(1, :), z_WS(2, :), 'c.');
% for t = 1:T_total

% 	[pEV, cEV] = plotCar(z_WS(1, t), z_WS(2, t), z_WS(3, t), EV.width, EV.length, plt_ops);
% 	hold on

% 	plt_ob1 = plot(Obs{1, t}, 'color', 'black');
% 	hold on
% 	plt_ob2 = plot(Obs{2, t}, 'color', 'black');
% 	hold on
% 	plt_ob3 = plot(Obs{3, t}, 'color', 'black');
% 	hold on
% 	axis(map_dim);
% 	pause(0.05)

% 	delete([pEV, cEV])

% 	delete(plt_ob1)
% 	delete(plt_ob2)
% 	delete(plt_ob3)
% end

%% Plot Final solution
plt_EV_opt = plot(EV.traj(1, :), EV.traj(2, :), 'b');
for t = 1:T_total

	% Stop plotting early when drive out of frame
	if EV.traj(1, t) > EV.x_max
		break
	end

	[pEV, cEV] = plotCar(EV.traj(1, t), EV.traj(2, t), EV.traj(3, t), EV.width, EV.length, plt_ops);
	hold on

	plt_ob1 = plot(Obs{1, t}, 'color', 'black');
	hold on
	plt_ob2 = plot(Obs{2, t}, 'color', 'black');
	hold on
	plt_ob3 = plot(Obs{3, t}, 'color', 'black');
	hold on
	axis(map_dim);
	pause(0.05)

	delete([pEV, cEV])

	delete(plt_ob1)
	delete(plt_ob2)
	delete(plt_ob3)
end
delete(plt_EV_opt)