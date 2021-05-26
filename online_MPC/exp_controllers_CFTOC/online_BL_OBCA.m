clear('all');
close('all');
clc

addpath('../dynamics')

%% Load testing data
% uiopen('load')
exp_num = 8;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)
load('traj_data.mat', 'parking_lot', 'static_object_list')

fig = figure('units','normalized','outerposition',[0 0 1 1]);

%% Plot parking lot
parking_lines = squeeze(parking_lot(exp_num, :, :));

map_offset = [210, 285];

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

%%
N = 20; % Prediction horizon
dt = EV.dt; % Time step
T = length(TV.t); % Length of data
v_ref = EV.ref_v; % Reference velocity
y_ref = EV.ref_y; % Reference y

% Clear the traj and inputs field
EV.traj = EV.traj(:, 1);
EV.inputs = [];

all_feas = {};
all_z_ref = {};
all_z_opt = {};
all_TV_pred = {};

map_dim = [-30 30 -10 10];

p_EV = [];
l_EV = [];

p_TV = [];
l_TV = [];

EV_plt_opts.circle = false;
EV_plt_opts.frame = true;
EV_plt_opts.color = 'b';
EV_plt_opts.alpha = 0.5;

TV_plt_opts.circle = false;
TV_plt_opts.frame = true;
TV_plt_opts.color = 'g';
TV_plt_opts.alpha = 0.5;

F(T-N) = struct('cdata',[],'colormap',[]);

for i = 1:T-N
	delete(p_EV)
	delete(l_EV)

	delete(p_TV)
	delete(l_TV)

	% Get x, y, heading, and velocity from ego vehicle at current timestep
    EV_x  = EV.traj(1, end);
    EV_y  = EV.traj(2, end);
    EV_th = EV.traj(3, end);
    EV_v  = EV.traj(4, end);

    EV_curr = [EV_x; EV_y; EV_th; EV_v*cos(EV_th); EV_v*sin(EV_th)];

    % Get x, y, heading, and velocity from target vehicle over prediction
    % horizon
    TV_x = TV.x(i:i+N);
    TV_y = TV.y(i:i+N);
    TV_th = TV.heading(i:i+N);
    TV_v = TV.v(i:i+N);
    
    TV_pred = [TV_x, TV_y, TV_th, TV_v.*cos(TV_th), TV_v.*sin(TV_th)]';


    EV_x_ref = EV_x + [0:N]*dt*v_ref;
    EV_v_ref = v_ref*ones(1, N+1);
    EV_y_ref = zeros(1, length(EV_x_ref));
    EV_h_ref = zeros(1, length(EV_x_ref));
    z_ref = [EV_x_ref; EV_y_ref; EV_h_ref; EV_v_ref];
    if ~isfield(EV, 'z_opt')
        EV.z_opt = z_ref;
        EV.u_opt = zeros(2, N);
    end

    z0 = EV.traj(:, end);
    hyp.z_WS = [EV.z_opt(:, 2:end), EV.z_opt(:, end)];
    hyp.u_WS = [EV.u_opt(:, 2:end), EV.u_opt(:, end)];

    [EV.z_opt, EV.u_opt, feas] = hobca_CFTOC(z0, N, hyp, TV_pred, z_ref, EV);

    all_feas{i} = feas;
	all_z_ref{i} = z_ref;
	all_z_opt{i} = EV.z_opt;
	all_TV_pred{i} = TV_pred;

    EV.traj = [EV.traj, EV.z_opt(:, 2)];
    EV.inputs = [EV.inputs, EV.u_opt(:, 1)];

    % Plotting
    [p_EV, l_EV] = plotCar(EV_x, EV_y, EV_th, EV.width, EV.length, EV_plt_opts);

    for j = 1:N+1
    	if j == 1
    		TV_plt_opts.alpha = 0.5;
            TV_plt_opts.frame = true;
        else
        	TV_plt_opts.alpha = 0;
            TV_plt_opts.frame = false;
        end

        [p, l] = plotCar(TV_x(j), TV_y(j), TV_th(j), TV.width, TV.length, TV_plt_opts);
        p_TV = [p_TV, p];
        l_TV = [l_TV, l];

        l_TV = [l_TV plot(EV.z_opt(1, j), EV.z_opt(2, j), 'd', 'color', 'b')];
    end

    axis equal
    axis(map_dim);

    pause(0.05)
    % input('Any Key')

    F(i) = getframe(fig);

end

%% Save Movie
if ~isfolder('../../movies/')
    mkdir('../../movies')
end

movie_name = "BL_OBCA";
[file,path] = uiputfile(sprintf('../../movies/%s_Exp%d_%s.mp4', ...
                    movie_name, exp_num, datestr(now,'yyyy-mm-dd_HH-MM')));

v = VideoWriter([path, file], 'MPEG-4');
v.FrameRate = 10;
open(v);
writeVideo(v,F);
close(v);