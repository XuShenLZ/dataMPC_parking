%% Time invariant obstacle
% Use hybrid a* as the warm start
% Then OBCA
close('all');
clear('all');
clc

%% Load data
load('traj_data.mat');

%% Experiment number 
exp_num = 2;

%% Extract data
TV.dims   = ego_dims{exp_num};
TV.width  = TV.dims.width;
TV.length = TV.dims.length;

TV.traj    = ego_trajectory{exp_num};
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
line_x = parking_lines(:, 2);
line_y = parking_lines(:, 1);
line_w = parking_lines(:, 4);
line_h = parking_lines(:, 3);
parking_lines = [line_x, line_y, line_w, line_h];

static_objs = squeeze(static_object_list(exp_num, :, :));
% Flip x and y
obj_x = static_objs(:, 2);
obj_y = static_objs(:, 1);
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
	if static_obj(2) < 275 || static_obj(2) > 295
		continue
	end
	static_obj(1) = static_obj(1) - static_obj(3)/2;
	static_obj(2) = static_obj(2) - static_obj(4)/2;
	rectangle('Position', static_obj, 'FaceColor', [211,211,211]/255, 'EdgeColor', 'none')
	hold on
end
axis equal
% x_l, x_u, y_l, y_u
map_dim = [180 240 275 295];
% First dimension (y) and second dimension (x)
map_size = [map_dim(4)-map_dim(3), map_dim(2)-map_dim(1)];
axis(map_dim);

%% Object
% Parking spots
spots_u = Polyhedron('V', [183, 288.5; 183+53, 288.5; 183, 288.5+5.5; 183+53, 288.5+5.5]);
spots_l = Polyhedron('V', [183,   276; 183+53,   276; 183,   276+5.5; 183+53,   276+5.5]);

spots_u_plt = plot(spots_u, 'color', 'black', 'edgecolor', 'black');
hold on
spots_l_plt = plot(spots_l, 'color', 'black', 'edgecolor', 'black');
hold on

% Center Object
Obs{1} = Polyhedron('V', [210, 283; 210+2, 283; 210, 283+2; 210+2, 283+2]);
plot(Obs{1}, 'color', 'black')
hold on
axis(map_dim);

%% Get frame and transformation
frame = getframe;
img = rgb2gray(frame.cdata);
binary_img = img < 128;

% First dimension (y) and second dimension (x)
map2img_scale = size(img) ./ map_size;

%% EV startPose and goalPose
EV.length = TV.length;
EV.width  = TV.width;
EV.L = 0.6 * EV.length;
EV.offset = EV.L / 2;
EV.G = [1 0; -1 0; 0 1; 0 -1];
EV.g = [EV.length/2; EV.length/2; EV.width/2; EV.width/2];

EV.startPose = [200; 285; 0];
EV.goalPose  = [220; 285; 0];

EV.start_v = 2;
EV.goal_v  = 2;

img_startPose = EV.startPose;
img_startPose(1:2) = (img_startPose(1:2) - [map_dim(1); map_dim(3)]) .* map2img_scale';

img_goalPose = EV.goalPose;
img_goalPose(1:2) = (img_goalPose(1:2) - [map_dim(1); map_dim(3)]) .* map2img_scale';

%% Hybrid A*
ops.inflate_radius = mean(map2img_scale) * 1.5;
ops.MinTurningRadius = mean(map2img_scale) * 1;
ops.MotionPrimitiveLength = ops.MinTurningRadius * pi / 4;
ops.InterpolationDistance = mean(map2img_scale) * 0.2;
ops.ReverseCost = 1e7;
ops.length = mean(map2img_scale) * EV.length;
ops.width = mean(map2img_scale) * EV.width;

[img_refPath, planner] = hybrid_A_star(binary_img, img_startPose, img_goalPose, ops);
refPath = img_refPath;
refPath(:, 1:2) = img_refPath(:, 1:2) ./ map2img_scale + [map_dim(1), map_dim(3)];

N = size(refPath, 1) - 1;

%% Smooth path
refPath(:, 3) = refPath(:, 3) * 180 / pi;
[refPath, directions] = smoothPathSpline(refPath, ones(N+1, 1), N+1);
refPath(:, 3) = wrapToPi(refPath(:, 3) * pi / 180);
refPath = refPath';

%% Plot back
figure(fig_pl)
plot(refPath(1,:), refPath(2,:), 'r.', 'linewidth', 2)

%% Plot cars
for k = 1:N+1
	plt_ops.color = 'red';
	plt_ops.alpha = 0.5;
	plt_ops.circle = false;
	[pEV, cEV] = plotCar(refPath(1, k), refPath(2, k), refPath(3, k), EV.width, EV.length, plt_ops);
	hold on

	pause(0.1)
	delete([pEV, cEV])
end

%% HOBCA - Obstacles

% % spots_l
Obs{2} = Polyhedron('A', [0 1], 'b', (276+5.5));

% spots_u
Obs{3} = Polyhedron('A', [0 -1], 'b', -289);

% delete([spots_u_plt, spots_l_plt])

%% HOBCA - DualMultWS
v0 = EV.start_v;
dt = 0.1;
v_WS = [v0, vecnorm( diff(refPath(1:2,:), 1, 2) ) / dt];
z_WS = [refPath; v_WS];

[mu_WS, lambda_WS] = DualMultWS(N, Obs, EV, z_WS);

%% HOBCA - WS input
delta_WS = atan( diff(z_WS(3,:)) ./ v_WS(1:end-1) * EV.L / dt );

a_WS = diff(v_WS) / dt;
u_WS = [delta_WS; a_WS];

%% Model Construction
[z_opt, u_opt, mu_opt, lambda_opt] = OBCA(N, dt, Obs, EV, z_WS, u_WS, mu_WS, lambda_WS);

%% Check constraints
feasible = check_constr(N, dt, Obs, EV, z_opt, u_opt, mu_opt, lambda_opt);
constr_name = ["Dual", "State", "Input", "Dynamic", "Obstacle"];
for i = 1:length(constr_name)
	fprintf('%s constraints feasibility: %d \n', constr_name(i), feasible(i))
end

%% Plot result
opt_plt = plot(z_opt(1,:), z_opt(2,:), 'b.', 'linewidth', 2);
hold on

%% Transformation
z_center = z_opt;
for k = 1:N+1
	z_center(1, k) = z_opt(1, k) + EV.offset * cos(z_opt(3, k));
	z_center(2, k) = z_opt(2, k) + EV.offset * sin(z_opt(3, k));
end

%% Plot car
for k = 1:N+1
	plt_ops.color = 'blue';
	plt_ops.alpha = 0.5;
	plt_ops.circle = false;
	[pEV, cEV] = plotCar(z_center(1, k), z_center(2, k), z_center(3, k), EV.width, EV.length, plt_ops);
	hold on

	pause(0.1)
	delete([pEV, cEV])
end