clear('all');
close('all');
clc

addpath('../nominal_MPC')

%% Load testing data
% uiopen('load')
exp_num = 30;
exp_file = strcat('../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

%% Load strategy prediction model
model_name = 'nn_strategy_TF-trainscg_h-40_AC-tansig_ep2000_CE0.17453_2020-08-04_15-42';
model_file = strcat('../models/', model_name, '.mat');
load(model_file)

%%
N = 20; % Prediction horizon
dt = EV.dt; % Time step
T = length(TV.t); % Length of data
v_ref = EV.ref_v; % Reference velocity
y_ref = EV.ref_y; % Reference y
r = sqrt(EV.width^2 + EV.length^2)/2; % Collision buffer radius

% Make a copy of EV as the optimal EV, and naive EV
OEV = EV;
NEV = EV;
HEV = EV;

% Clear the traj and inputs field
EV.traj = EV.traj(:, 1);
EV.inputs = [];

NEV.traj = NEV.traj(:, 1);
NEV.inputs = [];

HEV.traj = HEV.traj(:, 1);
HEV.inputs = [];

OEV_plt_opts.circle = false;
OEV_plt_opts.frame = false;
OEV_plt_opts.color = 'g';
OEV_plt_opts.alpha = 0.5;

NEV_plt_opts.circle = false;
NEV_plt_opts.frame = false;
NEV_plt_opts.color = 'm';
NEV_plt_opts.alpha = 0.5;

HEV_plt_opts.circle = false;
HEV_plt_opts.frame = false;
HEV_plt_opts.color = 'c';
HEV_plt_opts.alpha = 0.3;

EV_plt_opts.circle = true;
EV_plt_opts.frame = true;
EV_plt_opts.color = 'b';
EV_plt_opts.alpha = 0.5;

TV_plt_opts.circle = false;
TV_plt_opts.color = 'y';

cmap = jet(N+1);

map_dim = [-30 30 -10 10];
p_EV = [];
l_EV = [];
p_OEV = [];
l_OEV = [];
p_NEV = [];
l_NEV = [];
p_HEV = [];
l_HEV = [];
p_TV = [];
l_TV = [];
t_EV_ref = [];
t_Y = [];
t_hyp_feas = [];
t_niv_feas = [];
t_h_feas = [];
p_online = [];

phi = linspace(0, 2*pi, 200);
coll_bound_x = zeros(1, 200);
coll_bound_y = zeros(1, 200);
for i = 1:length(phi)
    [x_b, y_b, ~, ~] = get_collision_boundary_point(0, 0, phi(i), TV.width, TV.length, r);
    coll_bound_x(i) = x_b;
    coll_bound_y(i) = y_b;
end

R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

F(T-N) = struct('cdata',[],'colormap',[]);

fig = figure('units','normalized','outerposition',[0 0 1 1]);

ax1 = subplot(2,1,1);

ax2 = subplot(2,1,2);
L_line = animatedline(ax2, 'color', '#0072BD', 'linewidth', 2);
R_line = animatedline(ax2, 'color', '#D95319', 'linewidth', 2);
Y_line = animatedline(ax2, 'color', '#77AC30', 'linewidth', 2);
legend('Left', 'Right', 'Yield')
prob_dim = [1 T-N -0.2 1.2];
axis(prob_dim)
grid on

for i = 1:T-N
    delete(p_EV)
    delete(l_EV)

    delete(p_OEV)
    delete(l_OEV)

    delete(p_NEV)
    delete(l_NEV)

    delete(p_HEV)
    delete(l_HEV)

    delete(p_TV)
    delete(l_TV)

    delete(t_EV_ref)

    delete(t_Y)
    delete(t_hyp_feas)
    delete(t_niv_feas)
    delete(t_h_feas)
    
    % Get x, y, heading, and velocity from ego vehicle at current timestep
    EV_x  = EV.traj(1, end);
    EV_y  = EV.traj(2, end);
    EV_th = EV.traj(3, end);
    EV_v  = EV.traj(4, end);

    if EV_x > map_dim(2)
        F(i:end) = [];
        break
    end
    
    EV_curr = [EV_x; EV_y; EV_th; EV_v*cos(EV_th); EV_v*sin(EV_th)];
    
    % Get x, y, heading, and velocity from target vehicle over prediction
    % horizon
    TV_x = TV.x(i:i+N);
    TV_y = TV.y(i:i+N);
    TV_th = TV.heading(i:i+N);
    TV_v = TV.v(i:i+N);
    
    TV_pred = [TV_x, TV_y, TV_th, TV_v.*cos(TV_th), TV_v.*sin(TV_th)]';
    
    % Get target vehicle trajectory relative to ego vehicle state
    rel_state = TV_pred - EV_curr;
    
    % Predict strategy to use based on relative prediction of target
    % vehicle
    X = reshape(rel_state, [], 1);
    score = net(X);
    [~, max_idx] = max(score);
    if max_idx == 1
        Y = "Left";
    elseif max_idx == 2
        Y = "Right";
    else
        Y = "Yield";
    end

    addpoints(L_line, i, score(1));
    addpoints(R_line, i, score(2));
    addpoints(Y_line, i, score(3));
    
    % Generate reference trajectory
    EV_x_ref = EV_x + [0:N]*dt*v_ref;
    EV_y_ref = zeros(1, length(EV_x_ref));
    
    % Check which points along the reference trajectory would result in
    % collision. Collision is defined as the reference point at step k 
    % being contained in O_TV(k) + B(r), where O_TV(k) is the region
    % occupied by the target vehicle at step k along the prediction horizon
    % and B(r) is the 2D ball with radius equal to the collision buffer
    % radius of the ego vehicle
    horizon_collision = [];
    for j = 1:N+1
        ref = [EV_x_ref(j); EV_y_ref(j)];
        collision = check_collision(ref, TV_x(j), TV_y(j), TV_th(j), TV.width, TV.length, r);
        horizon_collision = [horizon_collision, collision];
        if collision
            if max_idx == 1
                dir = [0; 1];
            elseif max_idx == 2
                dir = [0; -1];
            else
                dir = [EV_x-TV_x(j); EV_y-TV_y(j)];
                dir = dir/(norm(dir));
            end
            [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp(ref, dir, TV_x(j), TV_y(j), TV_th(j), TV.width, TV.length, r);
            hyp(j).w = hyp_w;
            hyp(j).b = hyp_b;
            hyp(j).pos = hyp_xy;
        else
            hyp(j).w = [];
            hyp(j).b = [];
            hyp(j).pos = [];
        end
    end

    % Parallel Online Controller
    % =======
    zz0{1} = EV.traj(:, end);
    zz0{2} = NEV.traj(:, end);
    zz_ref{1} = [EV_x_ref; EV_y_ref; zeros(1, N+1); v_ref*ones(1, N+1)];
    zz_ref{2} = [NEV.traj(1, end) + [0:N]*dt*v_ref; ...
                 zeros(2, N+1); 
                 v_ref*ones(1, N+1)];
    zz_opt = cell(1,2);
    uu_opt = cell(1,2);
    par_feas = zeros(1,2);

    parfor j = 1:2
        z0 = zz0{j};
        z_ref = zz_ref{j};
        if j == 1
            [zz_opt{j}, uu_opt{j}, par_feas(j)] = hpp_CFTOC(z0, N, hyp, Y, z_ref, EV);
            if ~par_feas(j)
                warning('HPP Not Feasible')
            end
        else
            [zz_opt{j}, uu_opt{j}, par_feas(j)] = niv_CFTOC(z0, N, TV_pred, r, z_ref, NEV);
            if ~par_feas(j)
                warning('Naive Not Feasible')
            end
        end
    end
    feas = par_feas(1);
    z_opt = zz_opt{1};
    u_opt = uu_opt{1};
    EV.traj = [EV.traj, z_opt(:, 2)];
    EV.inputs = [EV.inputs, u_opt(:, 1)];

    feas_niv = par_feas(2);
    z_niv = zz_opt{2};
    u_niv = uu_opt{2};
    NEV.traj = [NEV.traj, z_niv(:, 2)];
    NEV.inputs = [NEV.inputs, u_niv(:, 1)];

    % Online HOBCA
    z0_h = HEV.traj(:, end);
    z_ref_h = [z0_h(1) + [0:N]*dt*v_ref; ...
                 zeros(2, N+1); 
                 v_ref*ones(1, N+1)];
    [z_h, u_h, feas_h] = online_HOBCA(z0_h, N, TV_pred, r, z_ref_h, HEV);
    if ~feas_h
        warning('HOBCA Not Feasible')
    end
    HEV.traj = [HEV.traj, z_h(:, 2)];
    HEV.inputs = [HEV.inputs, u_h(:, 1)];
    % ============

    % Sequencial Online Controller
    % ============ 
    % z0 = EV.traj(:, end);
    % z_ref = [EV_x_ref; EV_y_ref; zeros(1, N+1); v_ref*ones(1, N+1)];
    % [z_opt, u_opt, feas] = hpp_CFTOC(z0, N, hyp, Y, z_ref, EV);
    % if ~feas
    %     warning('Not Optimal')
    % end
    % EV.traj = [EV.traj, z_opt(:, 2)];
    % EV.inputs = [EV.inputs, u_opt(:, 1)];

    % % Naive Online Controller
    % z0_niv = NEV.traj(:, end);
    % z_ref_niv = [z0_niv(1) + [0:N]*dt*v_ref; ...
    %              zeros(2, N+1); 
    %              v_ref*ones(1, N+1)];
    % [z_niv, u_niv, feas_niv] = niv_CFTOC(z0_niv, N, TV_pred, r, z_ref_niv, NEV);
    % if ~feas_niv
    %     warning('Not Optimal')
    % end
    % NEV.traj = [NEV.traj, z_niv(:, 2)];
    % NEV.inputs = [NEV.inputs, u_niv(:, 1)];
    % ==============

    % Plot
    axes(ax1);
    t_Y = text(-25, 8, sprintf('Strategy: %s', Y), 'color', 'k');
    hold on
    t_hyp_feas = text(-25, 6, sprintf('HPP Online MPC feas: %d', feas), 'color', 'b');
    hold on
    t_niv_feas = text(-25, 4, sprintf('Naive Online MPC feas: %d', feas_niv), 'color', 'm');
    hold on
    t_h_feas = text(-25, 2, sprintf('HOBCA Online MPC feas: %d', feas_h), 'color', 'c');
    hold on

    [p_OEV, l_OEV] = plotCar(OEV.traj(1, i), OEV.traj(2, i), OEV.traj(3, i), OEV.width, OEV.length, OEV_plt_opts);
    [p_NEV, l_NEV] = plotCar(NEV.traj(1, i), NEV.traj(2, i), NEV.traj(3, i), NEV.width, NEV.length, NEV_plt_opts);
    [p_EV, l_EV] = plotCar(EV_x, EV_y, EV_th, EV.width, EV.length, EV_plt_opts);
    [p_HEV, l_HEV] = plotCar(HEV.traj(1, i), HEV.traj(2, i), HEV.traj(3, i), HEV.width, HEV.length, HEV_plt_opts);

    
    p_TV = [];
    l_TV = [];
    for j = 1:N+1
%         TV_plt_opts.alpha = 1 - 0.7*((j-1)/(N));
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
        
        if ~isempty(hyp(j).w)
            coll_bound_global = R(TV_th(j))*[coll_bound_x; coll_bound_y] + [TV_x(j); TV_y(j)];
            l_TV = [l_TV plot(coll_bound_global(1,:), coll_bound_global(2,:), 'color', cmap(j,:))];
            if hyp(j).w(2) == 0
                hyp_x = [hyp(j).b, hyp(j).b];
                hyp_y = [map_dim(3), map_dim(4)];
            else
                hyp_x = [map_dim(1), map_dim(2)];
                hyp_y = (-hyp(j).w(1)*hyp_x+hyp(j).b)/hyp(j).w(2);
            end
            l_TV = [l_TV plot(hyp_x, hyp_y, 'color', cmap(j,:))];
            l_TV = [l_TV plot([EV_x_ref(j) hyp(j).pos(1)], [EV_y_ref(j) hyp(j).pos(2)], '-o', 'color', cmap(j,:))];
        end
        l_TV = [l_TV plot(EV_x_ref(j), EV_y_ref(j), 'o', 'color', cmap(j,:))];
        l_TV = [l_TV plot(z_niv(1, j), z_niv(2, j), 'x', 'color', 'm')];
        l_TV = [l_TV plot(z_opt(1, j), z_opt(2, j), 'd', 'color', cmap(j,:))];
        l_TV = [l_TV plot(z_h(1, j), z_h(2, j), '*', 'color', 'c')];
    end
    
    axis equal
    axis(map_dim);

    axes(ax2)
    drawnow
    axis auto
    axis(prob_dim);
    
    pause(0.05)
    F(i) = getframe(fig);
%     input('Any key')
end

%% Save Movie
if ~isfolder('../movies/')
    mkdir('../movies')
end

movie_name = "OnlineMPC";
[file,path] = uiputfile(sprintf('../movies/%s_Exp%d_%s.mp4', ...
                    movie_name, exp_num, datestr(now,'yyyy-mm-dd_HH-MM')));

v = VideoWriter([path, file], 'MPEG-4');
v.FrameRate = 10;
open(v);
writeVideo(v,F);
close(v);