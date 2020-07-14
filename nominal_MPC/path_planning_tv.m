close('all');
clear('all');
clc

%% Load data
load('traj_data.mat');
map_offset = [210, 285];

%% Experiment number 
exp_num = 1;

%% Extract data
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

%% Plot target vehicle
plt_TV = plot(TV.x, TV.y, 'r--', 'linewidth', 2);
hold on

%% Ego Vehicle Attributes and Reference Trajectory

dt = TV.t(2) - TV.t(1); % Should be 0.1s

EV.x_min = map_dim(1);
EV.x_max = map_dim(2);

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

%% Obstacles
T_total = 80;

N = 80;

ob_V0 = [0, -2; 2, -2; 0, 0; 2, 0];
ob_x0 = 15;
% ob_x0 = 0;
ob_v0 = -0.25;

for t = 1:(T_total+N)
	Obs{1, t} = Polyhedron('A', [0  1], 'b', -3.5);
	Obs{2, t} = Polyhedron('A', [0 -1], 'b', -3.5);

	if t <= T_total
		ob_v0 = -0.25;
		% ob_v0 = 0;
		ob_V = ob_V0 + [ob_x0 + t*ob_v0, 0];
	end

	Obs{3, t} = Polyhedron('V', ob_V);

	plt_ob1 = plot(Obs{1, t}, 'color', 'black');
	hold on
	plt_ob2 = plot(Obs{2, t}, 'color', 'black');
	hold on
	plt_ob3 = plot(Obs{3, t}, 'color', 'black');
	hold on
	pause(0.05)
	delete(plt_ob1)
	delete(plt_ob2)
	delete(plt_ob3)
end

%% Ego vehicle tracking
% Tracking horizon
EV.z0 = [-8; 0; 0; EV.ref_v];

EV.z_opt = zeros(4, N+1);
EV.u_opt = zeros(2, N);

EV.traj = EV.z0;
EV.inputs = [];

la = 1;

t = 0;

plt_ops.color = 'blue';
plt_ops.alpha = 0.5;
plt_ops.circle = false;

while EV.z0(1) <= EV.x_max && t < T_total
	t = t + 1;

	[pEV, cEV] = plotCar(EV.z0(1), EV.z0(2), EV.z0(3), EV.width, EV.length, plt_ops);
	hold on
	mpc_plt = plot(EV.z_opt(1,:), EV.z_opt(2,:), 'go');
	hold on
	plt_ob3 = plot(Obs{3, t}, 'color', 'black');
	hold on
	pause(0.01)

	fprintf('t / T_total = %d / %d \n', t, T_total)
	fprintf('x / x_max = %.2f / %.2f \n', EV.z0(1), EV.x_max)

	EV.ref_z = [EV.z0(1): dt*EV.ref_v : (EV.z0(1) + (N-1)*dt*EV.ref_v);
				EV.ref_y * ones(1, N);
				EV.ref_h * ones(1, N);
				EV.ref_v * ones(1, N)];

	% Compute the simple path tracking controller
	[EV.pf_z_opt, EV.pf_u_opt, feas] = track_controller(N, dt, EV);
	% plt = plot(EV.pf_z_opt(1, :), EV.pf_z_opt(2, :), 'm.');
	% pause(1);
	% delete(plt);

	if feas
		% Check the collision from the simple path tracking result
		[collide, EV.goalPose, EV.goal_v] = check_collision_tv(Obs, EV, N, t);
		plt = plot(EV.goalPose(1), EV.goalPose(2), 'rx');
		pause(1);
		delete(plt);
		disp(collide);
	else
		warning('Simple Tracking is not feasible! Exiting..');
		break
	end

	if any(collide)

		[z_WS, feas] = unicycleWS(t, N, dt, Obs, EV);

		if feas
			v_WS = z_WS(4, :);
		else
			% Select the mean 
			collide_k = floor(mean(find(collide == 1)));

			% Plot the obstacles
			figure(fig_pl)
			plt_ob1 = plot(Obs{1, t+collide_k}, 'color', 'black');
			hold on
			plt_ob2 = plot(Obs{2, t+collide_k}, 'color', 'black');
			hold on
			plt_ob3 = plot(Obs{3, t+collide_k}, 'color', 'black');
			hold on
			pause(0.05)

			%% Get frame and transformation
			frame = getframe;
			img = rgb2gray(frame.cdata);
			binary_img = img < 128;

			% First dimension (y) and second dimension (x)
			map2img_scale = size(img) ./ map_size;

			disp('Hybrid A* initiated');
			% Use hybrid A*
			img_startPose = EV.z0(1:3);
			img_startPose(1:2) = (img_startPose(1:2) - [map_dim(1); map_dim(3)]) .* map2img_scale';

			img_goalPose = EV.goalPose;
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

			if ~isempty(img_refPath)
				% If the hybrid A* is feasible
				refPath = img_refPath;
				refPath(:, 1:2) = img_refPath(:, 1:2) ./ map2img_scale + [map_dim(1), map_dim(3)];

				% Smooth the path and interpolate into N+1 poses
				refPath(:, 3) = refPath(:, 3) * 180 / pi;
				[refPath, directions] = smoothPathSpline(refPath, ones(size(refPath, 1), 1), N+1);
				refPath(:, 3) = wrapToPi(refPath(:, 3) * pi / 180);
				refPath = refPath';
			else
				% If hybrid A* is not feasible
				% Use the last opt solution as WS (extend the last element)
				refPath = [EV.z_opt(1:3, 2:end), EV.z_opt(1:3, end)];
			end

			v_WS = [EV.z0(4), vecnorm( diff(refPath(1:2,:), 1, 2) ) / dt];
			z_WS = [refPath; v_WS];
		end

		delta_WS = atan( diff(z_WS(3,:)) ./ v_WS(1:end-1) * EV.L / dt );

		a_WS = diff(v_WS) / dt;
		u_WS = [delta_WS; a_WS];

		% Warm start Dual Mult
		[mu_WS, lambda_WS] = DualMultWS_tv(t, N, Obs, EV, z_WS);

		% Reference Trajectories
		EV.ref_z = [linspace(EV.z0(1), EV.goalPose(1), N);
					EV.ref_y * ones(1, N);
					EV.ref_h * ones(1, N);
					EV.ref_v * ones(1, N)];

		% Solve for collision free traj
		[z_opt, u_opt, mu_opt, lambda_opt, feas] = OBCA_tv(t, N, dt, Obs, EV, z_WS, u_WS, mu_WS, lambda_WS);
		
		if ~feas
			warning('Opti is infeasible');
			% debug mode
			% Plot hybrid A* path
			figure(fig_pl)
			ws_plt = plot(z_WS(1,:), z_WS(2,:), 'r.', 'linewidth', 2);
			hold on

			opt_plt = plot(z_opt(1,:), z_opt(2,:), 'b.', 'linewidth', 2);
			hold on

			% fig_astar = figure;
			% show(planner)

			% Pause for debug
			keyboard

			% close(fig_astar)
			delete(ws_plt)
			delete(opt_plt)

			% Use the last opti result
			EV.z_opt = [EV.z_opt(:, 2:end), EV.z_opt(:, end)];
			EV.u_opt = [EV.u_opt(:, 2:end), zeros(2, 1)];
		else
			EV.z_opt = z_opt;
			EV.u_opt = u_opt;
		end

		% delete(plt_ob1)
		% delete(plt_ob2)
		% delete(plt_ob3)
		EV.all_z_WS{t}  = z_WS;
	else
		% No collision along the way
		% Just use the result of simple tracking
		EV.z_opt = EV.pf_z_opt;
		EV.u_opt = EV.pf_u_opt;

		EV.all_z_WS{t}  = EV.pf_z_opt;
	end

	EV.z0 = EV.z_opt(:, 2);
	EV.u0 = EV.u_opt(:, 1);
	
	EV.traj = [EV.traj, EV.z0];
	EV.inputs = [EV.inputs, EV.u0];

	EV.all_ref_z{t} = EV.ref_z;
	EV.all_collide{t} = collide;
	EV.all_z_opt{t} = EV.z_opt;
	EV.all_u_opt{t} = EV.u_opt;

	EV.all_pf_z_opt{t} = EV.pf_z_opt;
	EV.all_pf_u_opt{t} = EV.pf_u_opt;

	delete([pEV, cEV])
	delete(mpc_plt)
	delete(plt_ob3)

end