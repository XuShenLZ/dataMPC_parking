close('all');
clear('all');
clc

%% Load data
load('traj_data.mat');
map_offset = [210, 285];

%% Experiment number 
exp_num = 2;

%% Extract data
TV.dims   = ego_dims{exp_num};
TV.width  = TV.dims.width;
TV.length = TV.dims.length;

TV.traj    = ego_trajectory{exp_num};
TV.traj(:, 1:2) = TV.traj(:, 1:2) - map_offset;
TV.t       = TV.traj(:, 1);
% Flip x and y
TV.x       = TV.traj(:, 3);
TV.y       = TV.traj(:, 2);
% Flip x and y -> fix heading
TV.heading = -TV.traj(:, 4) + pi/2;

TV.v       = TV.traj(:, 5);
TV.yawDot  = TV.traj(:, 6);

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

N_total = length(TV.t); % Total number of time steps

%% Plot parking lot
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

%% Ego Vehicle Attributes and Reference Trajectory

dt = TV.t(2) - TV.t(1); % Should be 0.1s

% ref_x0 = -25;

EV.x_min = map_dim(1);
EV.x_max = map_dim(2);

EV.ref_v = 2; % 2m/s = 7.2 km/h
EV.ref_y = 0;
EV.ref_h = 0;
% EV.ref_x = ref_x0 : ref_v*dt : (ref_x0 + (N_total-1)*ref_v*dt);
% EV.ref_y = zeros(1, length(EV.ref_x));
% EV.ref_h = zeros(1, length(EV.ref_x));
% EV.ref_v = ref_v * ones(1, length(EV.ref_x));

EV.width = TV.width;
EV.length = TV.length;
EV.L = 0.6 * EV.length;
EV.offset = EV.L / 2;
EV.G = [1 0; -1 0; 0 1; 0 -1];
EV.g = [EV.length/2; EV.length/2; EV.width/2; EV.width/2];
P_EV = Polyhedron('A', EV.G, 'b', EV.g);
EV.V = P_EV.V;

% plot(EV.ref_x, EV.ref_y, '.')
% hold on

%% Obstacles
% spots_l
Obs{1} = Polyhedron('A', [0 1], 'b', -3.5);
plot(Obs{1}, 'color', 'black')
hold on

% spots_u
Obs{2} = Polyhedron('A', [0 -1], 'b', -3.5);
plot(Obs{2}, 'color', 'black')
hold on

% Obstacle
Obs{3} = Polyhedron('V', [0, -2; 2, -2; 0, 0; 2, 0]);
plot(Obs{3}, 'color', 'black')
hold on
axis(map_dim);

%% Get frame and transformation
frame = getframe;
img = rgb2gray(frame.cdata);
binary_img = img < 128;

% First dimension (y) and second dimension (x)
map2img_scale = size(img) ./ map_size;

%% Ego vehicle tracking
% Tracking horizon
EV.z0 = [-8; 0; 0; EV.ref_v];

EV.traj = EV.z0;
EV.inputs = [];

N = 20;

la = 1;
while EV.z0(1) <= EV.x_max
	fprintf('x / x_max = %.2f / %.2f \n', EV.z0(1), EV.x_max)

	[collide, deviate, goalPose] = check_collision(Obs, EV, la, N, dt, fig_pl);
	fprintf('collide: %d \ndeviate: %d \n', collide, deviate)

	if collide
		disp('Hybrid A* initiated');
		% Use hybrid A*
		img_startPose = EV.z0(1:3);
		img_startPose(1:2) = (img_startPose(1:2) - [map_dim(1); map_dim(3)]) .* map2img_scale';

		img_goalPose = goalPose;
		img_goalPose(1:2) = (img_goalPose(1:2) - [map_dim(1); map_dim(3)]) .* map2img_scale';

		ops.inflate_radius = mean(map2img_scale) * 1;
		ops.MinTurningRadius = mean(map2img_scale) * 1;
		ops.MotionPrimitiveLength = ops.MinTurningRadius * pi / 4;
		ops.InterpolationDistance = mean(map2img_scale) * 0.2;
		ops.ReverseCost = 1e7;
		ops.length = mean(map2img_scale) * EV.length;
		ops.width = mean(map2img_scale) * EV.width;

		% Compute Hybrid A* path
		[img_refPath, planner] = hybrid_A_star(binary_img, img_startPose, img_goalPose, ops);
		refPath = img_refPath;
		refPath(:, 1:2) = img_refPath(:, 1:2) ./ map2img_scale + [map_dim(1), map_dim(3)];

		N_astar = size(refPath, 1) - 1;

		refPath(:, 3) = refPath(:, 3) * 180 / pi;
		[refPath, directions] = smoothPathSpline(refPath, ones(N_astar+1, 1), N_astar+1);
		refPath(:, 3) = wrapToPi(refPath(:, 3) * pi / 180);
		refPath = refPath';

		% Plot hybrid A* path
		figure(fig_pl)
		astar_plt = plot(refPath(1,:), refPath(2,:), 'r.', 'linewidth', 2);
		hold on

		v_WS = [EV.z0(4), vecnorm( diff(refPath(1:2,:), 1, 2) ) / dt];
		z_WS = [refPath; v_WS];

		delta_WS = atan( diff(z_WS(3,:)) ./ v_WS(1:end-1) * EV.L / dt );

		a_WS = diff(v_WS) / dt;
		u_WS = [delta_WS; a_WS];

		% Warm start Dual Mult
		[mu_WS, lambda_WS] = DualMultWS(N_astar, Obs, EV, z_WS);

		% Reference Trajectories
		EV.ref_z = [linspace(EV.z0(1), goalPose(1), N_astar);
					EV.ref_y * ones(1, N_astar);
					EV.ref_h * ones(1, N_astar);
					EV.ref_v * ones(1, N_astar)];

		% Solve for collision free traj
		[z_opt, u_opt, mu_opt, lambda_opt] = OBCA_f(N_astar, dt, Obs, EV, z_WS, u_WS, mu_WS, lambda_WS);

		opt_plt = plot(z_opt(1,:), z_opt(2,:), 'b.', 'linewidth', 2);
		hold on

		EV.traj = [EV.traj, z_opt(:, 2:end)];
		EV.inputs = [EV.inputs, u_opt];

		EV.z0 = z_opt(:, end);
	
	else
		disp('Normal Path Tracking');
		EV.ref_z = [EV.z0(1): dt*EV.ref_v : (EV.z0(1) + (N-1)*dt*EV.ref_v);
					EV.ref_y * ones(1, N);
					EV.ref_h * ones(1, N);
					EV.ref_v * ones(1, N)];

		v_WS = [EV.z0(4), EV.ref_v * ones(1, N)];
		z_WS = [EV.z0, EV.ref_z];

		[mu_WS, lambda_WS] = DualMultWS(N, Obs, EV, z_WS);

		delta_WS = atan( diff(z_WS(3,:)) ./ v_WS(1:end-1) * EV.L / dt );
		a_WS = diff(v_WS) / dt;
		u_WS = [delta_WS; a_WS];

		[z_opt, u_opt, mu_opt, lambda_opt] = OBCA_f(N, dt, Obs, EV, z_WS, u_WS, mu_WS, lambda_WS);

		EV.z0 = z_opt(:, 2);
		EV.traj = [EV.traj, EV.z0];
		EV.inputs = [EV.inputs, u_opt(:, 1)];
	end

end

%% Plot car
for k = 1:length(EV.traj(1, :))
	plt_ops.color = 'red';
	plt_ops.alpha = 0.5;
	plt_ops.circle = false;
	[pEV, cEV] = plotCar(EV.traj(1, k), EV.traj(2, k), EV.traj(3, k), EV.width, EV.length, plt_ops);
	hold on

	pause(0.1)
	delete([pEV, cEV])
end