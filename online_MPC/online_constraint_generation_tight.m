close all
clear all

addpath('../nominal_MPC')

%% Load testing data
% uiopen('load')
exp_num = 1;
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
p_TV = [];
l_TV = [];
t_EV_ref = [];
t_Y = [];

phi = linspace(0, 2*pi, 200);
coll_bound_x = zeros(1, 200);
coll_bound_y = zeros(1, 200);
for i = 1:length(phi)
    [x_b, y_b, ~, ~] = get_collision_boundary_point(0, 0, phi(i), TV.width, TV.length, r);
    coll_bound_x(i) = x_b;
    coll_bound_y(i) = y_b;
end

R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

figure('Position', [50 50 900 600])
ax1 = subplot(2,1,1);
ax2 = subplot(2,1,2);

L_line = animatedline(ax2, 'color', '#0072BD', 'linewidth', 2);
R_line = animatedline(ax2, 'color', '#D95319', 'linewidth', 2);
Y_line = animatedline(ax2, 'color', '#77AC30', 'linewidth', 2);
legend('Left', 'Right', 'Yield')
prob_dim = [1 T-N -0.2 1.2];
axis(prob_dim)

start_k = 60;
for i = start_k:T-N
    delete(p_EV)
    delete(l_EV)
    delete(p_TV)
    delete(l_TV)
    delete(t_EV_ref)
    delete(t_Y)
    
    % Get x, y, heading, and velocity from ego vehicle at current timestep
    EV_x = EV.traj(1,i);
    EV_y = EV.traj(2,i);
    EV_th = EV.traj(3,i);
    EV_v = EV.traj(4,i);
    
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
%             bias_dir = [-1; 0];
%             bias_dir = [-cos(EV_th); -sin(EV_th)];
%             s = score(max_idx);
%             s = 1;
            [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp_tight(ref, dir, TV_x(j), TV_y(j), TV_th(j), ...
                TV.width, TV.length, r);
            hyp(j).w = hyp_w;
            hyp(j).b = hyp_b;
            hyp(j).pos = hyp_xy;
        else
            hyp(j).w = [];
            hyp(j).b = [];
            hyp(j).pos = [];
        end
    end
    
    % Plot
    subplot(2,1,1)
    if max_idx == 1
        Y = 'Left';
    elseif max_idx == 2
        Y = 'Right';
    else
        Y = 'Yield';
    end
    t_Y = text(EV_x, EV_y+1.5*r, sprintf('Strategy: %s', Y), 'color', 'k');
    hold on
    [p_EV, l_EV] = plotCar(EV_x, EV_y, EV_th, EV.width, EV.length, EV_plt_opts);
    
    p_TV = [];
    l_TV = [];
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
        
        if ~isempty(hyp(j).w)
            coll_bound_global = R(TV_th(j))*[coll_bound_x; coll_bound_y] + [TV_x(j); TV_y(j)];
            l_TV = [l_TV plot(coll_bound_global(1,:), coll_bound_global(2,:), 'color', cmap(j,:))];
            if abs(hyp(j).w(2)) <= 1e-8
                hyp_x = [hyp(j).b/hyp(j).w(1), hyp(j).b/hyp(j).w(1)];
                hyp_y = [map_dim(3), map_dim(4)];
            else
                hyp_x = [map_dim(1), map_dim(2)];
                hyp_y = (-hyp(j).w(1)*hyp_x+hyp(j).b)/hyp(j).w(2);
            end
            l_TV = [l_TV plot(hyp_x, hyp_y, 'color', cmap(j,:))];
            l_TV = [l_TV plot([EV_x_ref(j) hyp(j).pos(1)], [EV_y_ref(j) hyp(j).pos(2)], '-o', 'color', cmap(j,:))];
        end
        l_TV = [l_TV plot(EV_x_ref(j), EV_y_ref(j), 'o', 'color', cmap(j,:))];
    end
    
    axis equal
    axis(map_dim);
    
    addpoints(L_line, i, score(1));
    addpoints(R_line, i, score(2));
    addpoints(Y_line, i, score(3));
    drawnow
    
    pause(0.05)
%     input('Any key')
end