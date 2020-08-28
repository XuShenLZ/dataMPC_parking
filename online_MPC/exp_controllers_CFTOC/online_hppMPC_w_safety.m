clear('all');
close('all');
clc

addpath('../constraint_generation')
addpath('../utils')
addpath('../plotting')

%% Load testing data
% uiopen('load')
exp_num = 4;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

%% Load strategy prediction model
model_name = 'nn_strategy_TF-trainscg_h-40_AC-tansig_ep2000_CE0.17453_2020-08-04_15-42';
model_file = strcat('../../models/', model_name, '.mat');
load(model_file)

%%
N = 20; % Prediction horizon
dt = EV.dt; % Time step
T = length(TV.t); % Length of data
v_ref = EV.ref_v; % Reference velocity
y_ref = EV.ref_y; % Reference y
r = sqrt(EV.width^2 + EV.length^2)/2; % Collision buffer radius

hpp_control = HppController(N, EV);

% Instantiate safety controller
d_lim = [-0.35, 0.35];
a_lim = [-1, 1];
hpp_safety_control = safety_controller(dt, d_lim, a_lim);
hpp_ebrake_control = ebrake_controller(dt, d_lim, a_lim);
niv_safety_control = safety_controller(dt, d_lim, a_lim);

% ==== Filter Setup
V = 0.01 * eye(3);
W = 0.5 * eye(3);
Pm = 0.2 * eye(3);
score = ones(3, 1) / 3;

strategies = ["Left", "Right", "Yield"];

% Make a copy of EV as the optimal EV, and naive EV
OEV = EV;
NEV = EV;

% Clear the traj and inputs field
EV.traj = EV.traj(:, 1);
EV.inputs = [];

NEV.traj = NEV.traj(:, 1);
NEV.inputs = [];

OEV_plt_opts.circle = false;
OEV_plt_opts.frame = false;
OEV_plt_opts.color = 'g';
OEV_plt_opts.alpha = 0.5;

NEV_plt_opts.circle = false;
NEV_plt_opts.frame = false;
NEV_plt_opts.color = 'm';
NEV_plt_opts.alpha = 0.5;

EV_plt_opts.circle = false;
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
p_TV = [];
l_TV = [];
t_EV_ref = [];
t_Y = [];
t_niv_feas = [];
t_h_feas = [];

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

% fig = figure('units','normalized','outerposition',[0 0 1 1]);
fig = figure('Position', [50 50 1200 600]);

ax1 = axes('Position',[0.05 0.55 0.4 0.4]);
yline(3.5, '-.', 'color', '#7E2F8E', 'linewidth', 2)
hold on
yline(-3.5, '-.', 'color', '#7E2F8E', 'linewidth', 2)
axis equal
axis(map_dim);

ax2 = axes('Position',[0.05 0.05 0.4 0.4]);
L_line = animatedline(ax2, 'color', '#0072BD', 'linewidth', 2);
R_line = animatedline(ax2, 'color', '#D95319', 'linewidth', 2);
Y_line = animatedline(ax2, 'color', '#77AC30', 'linewidth', 2);
legend('Left', 'Right', 'Yield')
prob_dim = [1 T-N 0 1];
ylabel('Score')
axis(prob_dim)
grid on

% Plot inputs
ax_h_v = axes('Position',[0.5 0.75 0.45 0.2]);
h_v_l = animatedline(ax_h_v, 'color', '#0072BD', 'linewidth', 2);
ylabel('v')
axis([1 T-N -3 3])

ax_h_d = axes('Position',[0.5 0.52 0.45 0.2]);
h_d_l = animatedline(ax_h_d, 'color', '#0072BD', 'linewidth', 2);
ylabel('delta')
axis([1 T-N d_lim(1) d_lim(2)])

ax_h_a = axes('Position',[0.5 0.28 0.45 0.2]);
h_a_l = animatedline(ax_h_a, 'color', '#0072BD', 'linewidth', 2);
ylabel('a')
axis([1 T-N a_lim(1) a_lim(2)])

ax_h_s = axes('Position',[0.5 0.05 0.45 0.2]);
h_s_l = animatedline(ax_h_s, 'color', '#0072BD', 'linewidth', 2);
h_e_l = animatedline(ax_h_s, 'color', '#D95319', 'linewidth', 2, 'linestyle', '--');
legend('Safety', 'E-Brake')
ylabel('safety')
axis([1 T-N 0 1])

% fig_3 = figure('Position', [1350 50 600 600]);
% ax_n_d = subplot(3,1,1);
% n_d_l = animatedline(ax_n_d, 'color', '#0072BD', 'linewidth', 2);
% ylabel('delta')
% axis([1 T-N d_lim(1) d_lim(2)])
% title('Naive MPC')
% 
% ax_n_a = subplot(3,1,2);
% n_a_l = animatedline(ax_n_a, 'color', '#0072BD', 'linewidth', 2);
% ylabel('a')
% axis([1 T-N a_lim(1) a_lim(2)])
% 
% ax_n_s = subplot(3,1,3);
% n_s_l = animatedline(ax_n_s, 'color', '#0072BD', 'linewidth', 2);
% ylabel('safety')
% axis([1 T-N 0 1])

hpp_mpc_safety = false;
niv_mpc_safety = false;

for i = 1:T-N
    fprintf('=====================================\n')
    fprintf('Iteration: %i\n', i)
    
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
    if ~hpp_mpc_safety && EV_v*cos(EV_th) > 0
        rel_state = TV_pred - EV_curr;
    else
        tmp = [EV_x; EV_y; EV_th; 0; EV_v*sin(EV_th)];
        rel_state = TV_pred - tmp;
    end
    
    % Predict strategy to use based on relative prediction of target
    % vehicle
    X = reshape(rel_state, [], 1);
    score_z = net(X);

    % Filter
    [score, Pm] = score_KF(score, score_z, V, W, Pm);

    [~, max_idx] = max(score);

    addpoints(L_line, i, score(1));
    addpoints(R_line, i, score(2));
    addpoints(Y_line, i, score(3));
    
    % Generate reference trajectory
    yield = false;
%     if all( abs(rel_state(1, :)) > 10 )
    if all( abs(rel_state(1, :)) > 20 ) || rel_state(1,1) < -r
        % If it is still far away
        EV_x_ref = EV_x + [0:N]*dt*v_ref;
        EV_v_ref = v_ref*ones(1, N+1);
        hpp_mpc_safety = false;
        niv_mpc_safety = false;
        if all( abs(rel_state(1, :)) > 20 )
            fprintf('Cars are far away, tracking nominal reference velocity\n')
        end
        if rel_state(1,1) < -r
            fprintf('EV has passed TV, tracking nominal reference velocity\n')
        end
    elseif max(score) > 0.55 && max_idx < 3
        % If strategy is not yield discount reference velocity based on max
        % likelihood
        % EV_x_ref = EV_x + [0:N]*dt*v_ref*max(score);
        EV_x_ref = EV_x + [0:N]*dt*v_ref;
        EV_v_ref = max(score)*v_ref*ones(1, N+1);
%         EV_v_ref = v_ref*ones(1, N+1);
        hpp_mpc_safety = false;
        niv_mpc_safety = false;
        fprintf('Confidence: %g, threshold met, tracking score discounted reference velocity\n', max(score))
    else
        % If yield or not clear to left or right
        yield = true;
        EV_x_ref = EV_x + [0:N]*dt*v_ref;
        EV_v_ref = v_ref*ones(1, N+1);
        fprintf('Confidence: %g, threshold not met, setting yield to true\n', max(score))
    end
    
    % EV_x_ref = EV_x + [0:N]*dt*v_ref;
    EV_y_ref = zeros(1, length(EV_x_ref));
    EV_h_ref = zeros(1, length(EV_x_ref));
    z_ref = [EV_x_ref; EV_y_ref; EV_h_ref; EV_v_ref];
    if ~isfield(EV, 'z_opt')
        EV.z_opt = z_ref;
        EV.u_opt = zeros(2, N);
    end
    
    % Check which points along the reference trajectory would result in
    % collision. Collision is defined as the reference point at step k 
    % being contained in O_TV(k) + B(r), where O_TV(k) is the region
    % occupied by the target vehicle at step k along the prediction horizon
    % and B(r) is the 2D ball with radius equal to the collision buffer
    % radius of the ego vehicle
    
    % ======= Use the ref traj to detect collision
    z_detect = z_ref; % Use the ref to construct hpp
    horizon_collision = [];
    for j = 1:N+1
        ref = z_detect(1:2, j); 
        collision = check_collision(ref, TV_x(j), TV_y(j), TV_th(j), TV.width, TV.length, r);
        horizon_collision = [horizon_collision, collision];
    end

    % Lock the strategy if more than 3 steps are colliding
%     if sum(horizon_collision) >= 3
%         strategy_idx = last_idx;
%         strategy_lock = true;
%     else
%         strategy_idx = max_idx;
%         last_idx = max_idx;
%         strategy_lock = false;
%     end
    strategy_lock = false;
    strategy_idx = max_idx;

    for j = 1:N+1
        ref = z_detect(1:2, j);
        collision = horizon_collision(j);
        if collision
            if strategy_idx == 1
                dir = [0; 1];
            elseif strategy_idx == 2
                dir = [0; -1];
            else
                dir = [EV_x-TV_x(j); EV_y-TV_y(j)];
                dir = dir/(norm(dir));
            end
            % ==== Unbiased Normal HPP
            [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp(ref, dir, TV_x(j), TV_y(j), TV_th(j), TV.width, TV.length, r);
            % ==== Unbiased Tight HPP
            % [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp_tight(ref, dir, TV_x(j), TV_y(j), TV_th(j), ...
            %     TV.width, TV.length, r);
            % ==== Biased HPP
            % bias_dir = [-1; 0];
            % % bias_dir = [-cos(EV_th); -sin(EV_th)];
            % s = score(max_idx);
            % [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp_score_bias(ref, dir, TV_x(j), TV_y(j), TV_th(j), ...
            %     TV.width, TV.length, r, bias_dir, s);
            % =====
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
    zz_ref{1} = z_ref;
    zz_ref{2} = [NEV.traj(1, end) + [0:N]*dt*v_ref; ...
                 zeros(2, N+1); 
                 v_ref*ones(1, N+1)];
    zz_opt = cell(1,2);
    uu_opt = cell(1,2);
    par_feas = zeros(1,2);
    
    if ~hpp_mpc_safety && yield
        % Compute the distance threshold for applying braking assuming max
        % decceleration is applied
        rel_vx = TV_v(1)*cos(TV_th(1)) - EV_v*cos(EV_th);
        min_ts = ceil(-rel_vx/abs(a_lim(1))/dt); % Number of timesteps requred for relative velocity to be zero
        v_brake = abs(rel_vx)+[0:min_ts]*dt*a_lim(1); % Velocity when applying max decceleration
%         brake_thresh = sum(abs(v_brake)*dt) + abs(TV_v(1)*cos(TV_th(1)))*(min_ts+1)*dt + 2*r; % Distance threshold for safety controller to be applied
        brake_thresh = sum(abs(v_brake)*dt) + 4*r;
        d = norm(TV_pred(1:2,1) - EV_curr(1:2), 2); % Distance between ego and target vehicles
        if  d <= brake_thresh
            % If distance between cars is within the braking threshold,
            % activate safety controller
            hpp_mpc_safety = true;
            fprintf('EV is within braking distance threshold, activating safety controller\n')
        end
    end

    % HPP MPC
    hpp_mpc_ebrake = false;
    err = 16; % Default: "User Terminated"
    if ~hpp_mpc_safety
        u0 = EV.u_opt(:, 1);
        [zz_opt{1}, uu_opt{1}, err] = hpp_control.solve(zz0{1}, u0, zz_ref{1}, hyp);
        if err
            % If HPP MPC is infeasible, activate ebrake controller
            hpp_mpc_ebrake = true;
            fprintf('HPP not feasible, activating emergency brake\n')
        else
            EV.z_opt = zz_opt{1};
            EV.u_opt = uu_opt{1};
        end
    end
    
    if hpp_mpc_safety
        hpp_safety_control = hpp_safety_control.set_speed_ref(TV_v(1)*cos(TV_th(1)));
        [u_safe, hpp_safety_control] = hpp_safety_control.solve(zz0{1}, TV_pred, EV.inputs(:,end));
        % Assume safety control is applied for one time step then no
        % control action is applied for rest of horizon
        u_sol = [u_safe zeros(2,N-1)];
        z_sol = [zz0{1} zeros(4,N)];
        % Simulate this policy
        for j = 1:N
            z_sol(:,j+1) = bikeFE_CoG(z_sol(:,j), u_sol(:,j), EV.L, dt);
        end
        % Update variables used for HPP warmstart (this is important
        % because if we don't update this, when the safety controller is
        % deactivated, HPP will likely have a bad warmstart)
        zz_opt{1} = z_sol;
        uu_opt{1} = u_sol;
        EV.z_opt = zz_opt{1};
        EV.u_opt = uu_opt{1};
        fprintf('Applying safety control\n')
    elseif hpp_mpc_ebrake
        [u_ebrake, hpp_ebrake_control] = hpp_ebrake_control.solve(zz0{1}, TV_pred, EV.inputs(:,end));
        % Assume ebrake control is applied for one time step then no
        % control action is applied for rest of horizon
        u_sol = [u_ebrake zeros(2,N-1)];
        z_sol = [zz0{1} zeros(4,N)];
        % Simulate this policy
        for j = 1:N
            z_sol(:,j+1) = bikeFE_CoG(z_sol(:,j), u_sol(:,j), EV.L, dt);
        end
        % Update variables used for HPP warmstart (this is important
        % because if we don't update this, when the safety controller is
        % deactivated, HPP will likely have a bad warmstart)
        zz_opt{1} = z_sol;
        uu_opt{1} = u_sol;
        EV.z_opt = zz_opt{1};
        EV.u_opt = uu_opt{1};
        fprintf('Applying ebrake control\n')
    end
    
    % Naive MPC
%     [zz_opt{2}, uu_opt{2}, par_feas(2)] = niv_CFTOC(zz0{2}, N, TV_pred, r, zz_ref{2}, NEV);
%     if ~par_feas(2)
%         % If naive MPC is infeasible, activate safety controller
%         niv_mpc_safety = true;
%         warning('Naive Not Feasible')
%     end
%     NEV.z_opt = zz_opt{2};
%     NEV.u_opt = uu_opt{2};
%     
%     if niv_mpc_safety
%         [uu_opt{2}, niv_safety_control] = niv_safety_control.solve(zz0{2}, TV_pred, NEV.inputs(:,end));
%         zz_opt{2} = [zz0{2} bikeFE_CoG(zz0{2}, uu_opt{2}, NEV.L, dt)];
%     end

    z_opt = zz_opt{1};
    u_opt = uu_opt{1};
    EV.traj = [EV.traj, z_opt(:, 2)];
    EV.inputs = [EV.inputs, u_opt(:, 1)];

%     feas_niv = par_feas(2);
%     z_niv = zz_opt{2};
%     u_niv = uu_opt{2};
%     NEV.traj = [NEV.traj, z_niv(:, 2)];
%     NEV.inputs = [NEV.inputs, u_niv(:, 1)];
    % ============
    
    addpoints(h_v_l, i, z_opt(4,2));
    addpoints(h_d_l, i, u_opt(1,1));
    addpoints(h_a_l, i, u_opt(2,1));
    addpoints(h_s_l, i, double(hpp_mpc_safety));
    addpoints(h_e_l, i, double(hpp_mpc_ebrake));
    
%     addpoints(n_d_l, i, u_niv(1,1));
%     addpoints(n_a_l, i, u_niv(2,1));
%     addpoints(n_s_l, i, double(niv_mpc_safety));

    % Delete lines and patches from last iteration
    delete(p_EV); delete(l_EV); delete(p_OEV); delete(l_OEV); 
%     delete(p_NEV); delete(l_NEV)
    delete(p_TV); delete(l_TV); delete(t_EV_ref); delete(t_Y); delete(t_h_feas); %delete(t_niv_feas)
    
    % Plot
    axes(ax1);
    t_Y = text(-29, 9, sprintf('Strategy: %s; Lock: %d', strategies(strategy_idx), strategy_lock), 'color', 'k');
    hold on
    if hpp_mpc_safety
        s_h = 'ON';
    else
        s_h = 'OFF';
    end
    if hpp_mpc_ebrake
        e_h = 'ON';
    else
        e_h = 'OFF';
    end
    t_h_feas = text(-29, 7, sprintf('HPP Online MPC: %s, Safety: %s, E-Brake: %s', yalmiperror(err), s_h, e_h), 'color', 'b');
    hold on
%     if niv_mpc_safety
%         s_n = 'ON';
%     else
%         s_n = 'OFF';
%     end
%     t_niv_feas = text(-30, 4, sprintf('Naive Online MPC feas: %d, Safety: %s', feas_niv, s_n), 'color', 'm');
%     hold on

    [p_OEV, l_OEV] = plotCar(OEV.traj(1, i), OEV.traj(2, i), OEV.traj(3, i), OEV.width, OEV.length, OEV_plt_opts);
%     [p_NEV, l_NEV] = plotCar(NEV.traj(1, i), NEV.traj(2, i), NEV.traj(3, i), NEV.width, NEV.length, NEV_plt_opts);
    [p_EV, l_EV] = plotCar(EV_x, EV_y, EV_th, EV.width, EV.length, EV_plt_opts);
    
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
            l_TV = [l_TV plot([z_detect(1, j) hyp(j).pos(1)], [z_detect(2, j) hyp(j).pos(2)], '-o', 'color', cmap(j,:))];
        end
        l_TV = [l_TV plot(EV_x_ref(j), EV_y_ref(j), 'o', 'color', cmap(j,:))];
%         if ~niv_mpc_safety
%             l_TV = [l_TV plot(z_niv(1, :), z_niv(2, :), 'x', 'color', 'm')];
%         end
        if ~hpp_mpc_safety
            l_TV = [l_TV plot(z_opt(1, j), z_opt(2, j), 'd', 'color', cmap(j,:))];
        end
    end
    axis equal
    axis(map_dim);

    axes(ax2)
    axis auto
    axis(prob_dim);
    drawnow

%     pause(0.05)
    F(i) = getframe(fig);
%     input('Any key')
end

%% Save Movie
if ~isfolder('../movies/')
    mkdir('../movies')
end

movie_name = "safety_hppMPC";
[file,path] = uiputfile(sprintf('../movies/%s_Exp%d_%s.mp4', ...
                    movie_name, exp_num, datestr(now,'yyyy-mm-dd_HH-MM')));

v = VideoWriter([path, file], 'MPEG-4');
v.FrameRate = 10;
open(v);
writeVideo(v,F);
close(v);