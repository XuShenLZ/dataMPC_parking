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
figure
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
axis([180 240 275 295]);

%% Plot TV traj

% for k = 1:length(TV.heading)
% 	plt_ops.color = 'green';
% 	plt_ops.alpha = 0.01;
% 	plotCar(TV.x(k), TV.y(k), TV.heading(k), TV.width, TV.length, plt_ops)
% 	hold on
% end

plot(TV.x, TV.y, 'b', 'linewidth', 2)
hold on

%% Ego Vehicle Attributes and Reference Trajectory
ref_v = 2; % 2m/s = 7.2 km/h
dt = TV.t(2) - TV.t(1); % Should be 0.1s

EV.ref_x0 = 185 + (-5);

EV.ref_x = EV.ref_x0 : ref_v*dt : EV.ref_x0 + (N_total-1)*ref_v*dt;
EV.ref_y = repmat(285, 1, length(EV.ref_x));
EV.ref_heading = zeros(1, length(EV.ref_x));

EV.width = TV.width;
EV.length = TV.length;

plot(EV.ref_x, EV.ref_y, '.')
hold on

%% Plot two cars
for k = 1:N_total
	plt_ops.color = 'green';
	plt_ops.alpha = 0.1;
	plt_ops.circle = true;
	[pTV, cTV] = plotCar(TV.x(k), TV.y(k), TV.heading(k), TV.width, TV.length, plt_ops);
	hold on

	plt_ops.color = 'red';
	plt_ops.alpha = 0.1;
	plt_ops.circle = true;
	[pEV, cEV] = plotCar(EV.ref_x(k), EV.ref_y(k), EV.ref_heading(k), EV.width, EV.length, plt_ops);
	hold on

	pause(0.05)
	delete([pTV, cTV, pEV, cEV])
end