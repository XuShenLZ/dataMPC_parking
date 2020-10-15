%% Load and visualize generated data
close('all');
% clear('all');
clc

%% Load .mat file
% uiopen('load')
exp_num = 4;
exp_file = strcat('../barc_data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

scale_ratio = 0.1101;

%% Construct Obs Polytopes
T_total = length(TV.t);
for t = 1:T_total
	% Static Obstacles
	Obs{1, t} = Polyhedron('A', [0  1], 'b', -3.5 * scale_ratio);
	Obs{2, t} = Polyhedron('A', [0 -1], 'b', -3.5 * scale_ratio);

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

%% Plot map
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
map_dim = [-30 30 -10 10] * 0.1101;
axis(map_dim);
% First dimension (y) and second dimension (x)
map_size = [map_dim(4)-map_dim(3), map_dim(2)-map_dim(1)];

%% Plot cars
plt_ops.color = 'blue';
plt_ops.alpha = 0.5;
plt_ops.circle = false;
plt_ops.frame = false;

plt_TV = plot(TV.x, TV.y, 'k--');
plt_EV_ref = plot(EV.ref_z(1, :), EV.ref_z(2, :), 'g.');
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
delete(plt_TV)
delete(plt_EV_ref)
delete(plt_EV_opt)