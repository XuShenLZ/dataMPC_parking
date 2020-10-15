%% Generating data in a parfor loop, more efficient
% Time varying Obstacles
% Warm Start + OBCA -> Speed Profile Control
% -> Emergency Break
close('all');
clear('all');
clc

%% Load data
load('traj_data.mat');
map_offset = [210, 285];

all_problem_type = cell(1, length(ego_dims));

%% BARC data
barc.L = 0.3239;
barc.width = 0.2794;
barc.length = 0.5334;

EV_L = ego_dims{1}.length * 0.6;
scale_ratio = barc.L / EV_L;

%% Determine the experiments to solve
generated_idxs = [];
for i = 1:length(ego_dims)
    if exist(sprintf('../barc_data/exp_num_%d.mat', i), 'file')
      generated_idxs = [generated_idxs, i];
    end
end
        
%% Experiment number 
parfor exp_num = 1:length(ego_dims)
    if ismember(exp_num, generated_idxs)
        fprintf('exp_num: %d is already generated. Skipping \n', exp_num)
        continue
    end
	fprintf('\n=====\nCurrent exp_num: %d\n', exp_num);

	%% Extract TV data
    TV = struct;
	TV.dims   = ego_dims{exp_num};
	TV.width  = barc.width;
	TV.length = barc.length;

	TV.traj    = ego_trajectory{exp_num};
	TV.t       = TV.traj(:, 1) * scale_ratio;
	% Flip x and y
	TV.x       = (TV.traj(:, 3) - map_offset(1)) * scale_ratio;
	TV.y       = (TV.traj(:, 2) - map_offset(2)) * scale_ratio;
	% Flip x and y -> fix heading
	TV.heading = -TV.traj(:, 4) + pi/2;

	TV.v       = TV.traj(:, 5) * scale_ratio;
	TV.yawDot  = TV.traj(:, 6);

	%% Extract Parking lot data
	parking_lines = squeeze(parking_lot(exp_num, :, :));
	% Flip x and y
	line_x = parking_lines(:, 2) - map_offset(1);
	line_y = parking_lines(:, 1) - map_offset(2);
	line_w = parking_lines(:, 4);
	line_h = parking_lines(:, 3);
	parking_lines = [line_x, line_y, line_w, line_h] * scale_ratio;

	static_objs = squeeze(static_object_list(exp_num, :, :));
	% Flip x and y
	obj_x = static_objs(:, 2) - map_offset(1);
	obj_y = static_objs(:, 1) - map_offset(2);
	obj_w = static_objs(:, 4);
	obj_h = static_objs(:, 3);
	static_objs = [obj_x, obj_y, obj_w, obj_h] * scale_ratio;

	%% Obstacles Definition
	T_total = length(TV.t);

	Obs = cell(3, T_total);
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

	%% Ego Vehicle Attributes and Reference Trajectory

	dt = TV.t(2) - TV.t(1); % Should be 0.1s
	EV = struct;
	EV.dt = dt;

	EV.x_min = -30 * scale_ratio;
	EV.x_max = 30 * scale_ratio;

	EV.a_max = 0.5;
	EV.delta_max = 0.6;

	EV.ref_v = 0.75; % 0.75 m/s
	EV.ref_y = 0;
	EV.ref_h = 0;

	EV.width = barc.width;
	EV.length = barc.length;
	EV.L = barc.L;
	EV.offset = EV.L / 2;
	EV.G = [1 0; -1 0; 0 1; 0 -1];
	EV.g = [EV.length/2; EV.length/2; EV.width/2; EV.width/2];
	P_EV = Polyhedron('A', EV.G, 'b', EV.g);
	EV.V = P_EV.V;

	collide_ref = false;

	% Decide the starting point
	for x0 = [-15, -10, -5, 0, 5, 10] * scale_ratio

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

	if collide_ref

		%% Solve for aggressive collision avoidance
		try

			[z_WS, feas_ws] = unicycleWS(0, T_total, dt, Obs, EV);

			if ~feas_ws
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

			problem_type = "Successful Maneuver";

		catch ME

			disp(ME.message);
			disp('Switched to speed control on ref path...');
			% Solve an optimal speed controller
			[z_opt, u_opt, feas_sc] = speed_controller(T_total, dt, Obs, EV);

			if feas_sc
				problem_type = "Speed Controller";
			else
				[z_opt, u_opt, feas_eb] = emergency_break(T_total, dt, EV, TV);

				collide_eb = check_collision_CFTOC(Obs, EV, z_opt, T_total);

				fprintf('Collision of Emergency Break: %d \n', collide_eb)

				if (~collide_eb) && feas_eb
					problem_type = "Emergency Break";
				else
					problem_type = "Infeasible";
				end

			end

		end

	else
		problem_type = "Collision Free";
		z_opt = EV.ref_z;
		u_opt = zeros(2, T_total);
	end

	EV.traj = z_opt;
	EV.inputs = u_opt;

	%% Save file
	parsave(exp_num, parking_lines, static_objs, TV, EV, problem_type);

	%% Save the log
	all_problem_type{exp_num} = problem_type;

end

%% Write the log 
log_filename = ['../barc_data/', datestr(now,'yyyy-mm-dd_HH-MM'), '.txt'];
fprintf('Writing the log...\n')
log_fileID = fopen(log_filename, 'a+');
for exp_num = 1:length(ego_dims)
	problem = load(sprintf('../barc_data/exp_num_%d.mat', exp_num), 'problem_type');
    problem_type = problem.problem_type;
    all_problem_type{exp_num} = problem_type;
	if ~isempty(problem_type)
		fprintf(log_fileID, '%s\n', problem_type);
	end
end
fclose(log_fileID);

%% parsave: function description
function parsave(exp_num, parking_lines, static_objs, TV, EV, problem_type)
	file_name = ['../barc_data/exp_num_', num2str(exp_num), '.mat'];
	save(file_name, ...
			'parking_lines', ...
			'static_objs', ...
			'TV', ...
			'EV', ...
			'problem_type')

	fprintf('Exp Num #%d is saved with type %s\n', exp_num, problem_type)
end