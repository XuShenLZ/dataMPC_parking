%% FSM_HOBCA_naive_fp: naive HOBCA
% exp_num (int, 1-486): The exp_num going to be evaluated
% data_gen (boolean): whether this is doing datagen or not
function [col, T_final] = tracking_fp(exp_num, data_gen)

    tExp = tic;

    sys_time = datestr(now,'yyyy-mm-dd_HH-MM');
    filename = 'tracking_fp';

    if data_gen
        if ~isfolder('../datagen/')
            mkdir('../datagen')
        end
    else
        if ~isfolder('../data/')
            mkdir('../data')
        end

        diary(sprintf('../data/%s_Exp%d_%s.txt', filename, exp_num, sys_time))
    end

    %% Load testing data
    exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
    load(exp_file)

    %%
    map_dim = [-5 5 -2 2];
    
    N = 50; % Prediction horizon
    dt = EV.dt; % Time step
    T = 1500; % Max time
    T_tv = length(TV.t); % Length of TV data
    
    TV_L = 0.5334;
    TV_W = 0.2794;
    
    % Instantiate ego vehicle car dynamics
    M = 10; % RK4 steps
    L_r = 0.1619;
    L_f = 0.1619;
    EV_dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);
    
    scaling_factor = (L_r+L_f)/TV.length;
    
    TV.length = TV_L;
    TV.width = TV_W;

    TV.x = TV.x*scaling_factor;
    TV.y = TV.y*scaling_factor;
    TV.v = TV.v*scaling_factor;

    % Extend TV traj to remain staionary after parked
    TV.x = [TV.x; TV.x(T_tv)*ones(N+1,1)];
    TV.y = [TV.y; TV.y(T_tv)*ones(N+1,1)];
    TV.heading = [TV.heading; TV.heading(T_tv)*ones(N+1,1)];
    TV.v = [TV.v; TV.v(T_tv)*ones(N+1,1)];
    
    n_z = 4;
    n_u = 2;
    
    lane_width = 1;
    
    % Instantiate obca controller
    % Q = diag([0.05 0.1 0.1 0.5]);
%     Q = diag([10 1 1 5]);
    Q = 10*eye(4);
    R = diag([1 1]);

    u_u = [0.5; 1.5];
    u_l = [-0.5; -2.5];
    du_u = [0.6; 5];
    du_l = [-0.6; -8];

    opt_params.name = 'tracking_solver';
    opt_params.N = N;
    opt_params.n_x = n_z;
    opt_params.n_u = n_u;
    opt_params.Q = Q;
    opt_params.R = R;
    opt_params.u_u = u_u;
    opt_params.u_l = u_l;
    opt_params.du_u = du_u;
    opt_params.du_l = du_l;
    opt_params.dynamics = EV_dynamics;
    opt_params.dt = dt;
    opt_params.optlevel = 3;

    if ~exist('forces_pro_gen_tracking', 'dir')
        mkdir('forces_pro_gen_tracking')
    end
    cd forces_pro_gen_tracking
    tracking_controller = tracking_controller_FP(true, opt_params);
    cd ..
    addpath('forces_pro_gen_tracking')

    % Data to save
    z_traj = zeros(n_z, T_tv+1);
    z_traj(:,1) = [TV.x(1); TV.y(1); TV.heading(1); TV.v(1)];
    u_traj = zeros(n_u, T_tv);

    z_preds = zeros(n_z, N+1, T_tv);
    u_preds = zeros(n_u, N, T_tv);
    z_refs = zeros(n_z, N+1, T_tv);

    sol_stats = cell(T_tv, 1);
    opt_solve_times = zeros(T_tv, 1);
    total_times = zeros(T_tv, 1);

    exp_params.exp_num = exp_num;
    exp_params.name = 'TRACKING MPC';
    exp_params.T = T;
    exp_params.T_tv = T_tv;
    exp_params.lane_width = lane_width;
    exp_params.map_dim = map_dim;

    exp_params.controller.N = N;
    exp_params.controller.Q = Q;
    exp_params.controller.R = R;
    exp_params.dynamics.dt = dt;
    exp_params.dynamics.M = M;
    exp_params.dynamics.L_r = L_r;
    exp_params.dynamics.L_f = L_f;
    exp_params.dynamics.n_z = n_z;
    exp_params.dynamics.n_u = n_u;


    %% 
    for i = 1:T_tv
        tIter = tic;
        fprintf('\n=================== Iteration: %i ==================\n', i)
        
        % Get x, y, heading, and velocity from target vehicle over prediction
        % horizon
        TV_x = TV.x(i:i+N);
        TV_y = TV.y(i:i+N);
        TV_th = TV.heading(i:i+N);
        TV_v = TV.v(i:i+N);
        
        z_ref = [TV_x, TV_y, TV_th, TV_v]';

        % HPP OBCA MPC
        % Initialize prediction guess for warm start
        if i == 1
            z_ws = z_ref;
            u_ws = zeros(n_u, N);
            u_prev = zeros(n_u, 1);
        else
            z_ws = [z_preds(:,2:end,i-1) EV_dynamics.f_dt(z_preds(:,end,i-1), u_preds(:,end,i-1))];
            u_ws = [u_preds(:,2:end,i-1) u_preds(:,end,i-1)];
            u_prev = u_traj(:,i-1);
        end

        [z_pred, u_pred, status_sol, tracking_controller] = tracking_controller.solve(z_traj(:,i), u_prev, z_ref, z_ws, u_ws);
        
        if ~status_sol.success
            error('Tracking MPC not feasible\n')
        else
            opt_solve_times(i) = status_sol.solve_time;
        end
        
        % Simulate system forward using first predicted input
        z_traj(:,i+1) = EV_dynamics.f_dt(z_traj(:,i), u_pred(:,1));
        u_traj(:,i) = u_pred(:,1);

        total_times(i) = toc(tIter);
        
        % Save data
        z_preds(:,:,i) = z_pred;
        u_preds(:,:,i) = u_pred;
        
        z_refs(:,:,i) = z_ref;

        sol_stats{i} = status_sol;
    end

    % Truncate the unused data buffer
    exp_params.T = T_tv;

    %% Save

    fprintf('\n=================== Complete ==================\n')

    if data_gen
        fpath = sprintf('../datagen/%s_Exp%d_%s.mat', filename, exp_num, sys_time);
    else
        diary off
        fprintf('Output log saved in: %s_Exp%d_%s.txt\n', filename, exp_num, sys_time);

        fpath = sprintf('../data/%s_Exp%d_%s.mat', filename, exp_num, sys_time);
    end

    save(fpath, 'exp_params', 'TV', ...
        'z_traj', 'u_traj', 'z_preds', 'u_preds', 'z_refs', ...
        'sol_stats', 'opt_solve_times', 'total_times')
    
    fprintf('Data saved in: %s\n', fpath)

    fprintf('Exp_num %d is done. Elapsed time is %d seconds.\n', exp_num, toc(tExp))


end