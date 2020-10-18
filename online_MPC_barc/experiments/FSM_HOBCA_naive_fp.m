%% FSM_HOBCA_naive_fp: naive HOBCA
% exp_num (int, 1-486): The exp_num going to be evaluated
% data_gen (boolean): whether this is doing datagen or not
function [col, T_final] = FSM_HOBCA_naive_fp(exp_num, data_gen)

    tExp = tic;

    sys_time = datestr(now,'yyyy-mm-dd_HH-MM');
    filename = 'FSM_HOBCA_naive_fp';

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
    
    N = 20; % Prediction horizon
    dt = EV.dt; % Time step
    T = 1500; % Max time
    T_tv = length(TV.t); % Length of TV data
    
    EV_L = 0.5334;
    EV_W = 0.2794;
    
    TV_L = 0.5334;
    TV_W = 0.2794;
    
    TV.length = TV_L;
    TV.width = TV_W;
    
    scaling_factor = EV_L/EV.length;
    
    EV.length = EV_L;
    EV.width = EV_W;
    
    x_max = 30*scaling_factor; % The right most x coordinate
    v_ref = EV.ref_v*scaling_factor; % Reference velocity
    y_ref = EV.ref_y*scaling_factor; % Reference y
    r = sqrt(EV_W^2 + EV_L^2)/2; % Collision buffer radius
    
    strategy_names = ["Left", "Right", "Yield"];
    
    TV.x = TV.x*scaling_factor;
    TV.y = TV.y*scaling_factor;
    TV.v = TV.v*scaling_factor;

    % Extend TV traj to remain staionary after parked
    TV.x(end+1:T) = TV.x(end);
    TV.y(end+1:T) = TV.y(end);
    TV.heading(end+1:T) = TV.heading(end);
    TV.v(end+1:T) = TV.v(end);
    
    EV_G = [1, 0; -1, 0; 0, 1; 0, -1];
    EV_g = [EV_L/2; EV_L/2; EV_W/2; EV_W/2];
    
    EV.traj = [EV.traj(1,:)*scaling_factor; EV.traj(2,:)*scaling_factor; EV.traj(3,:); EV.traj(4,:)*scaling_factor];
    
    n_z = 4;
    n_u = 2;

    % Instantiate ego vehicle car dynamics
    M = 10; % RK4 steps
    L_r = 0.1619;
    L_f = 0.1619;
    EV_dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);

    % Instantiate obca controller
    Q = [10 1 1 5];
    R = [1 1];
    R_d = [0.01, 0.01];
    
    z_u = [10; 10; 10; 1];
    z_l = [-10; -10; -10; -1];
    u_u = [0.35; 1];
    u_l = [-0.35; -1];
    du_u = [0.6; 5];
    du_l = [-0.6; -8];
    
    d_min = 0.01;
    % d_min = 0.001;

    n_obs = 3;
    n_ineq = [4,1,1];
    d_ineq = 2;

    tv_obs = cell(n_obs, N+1);
    lane_width = 1;
    for i = 1:N+1
        tv_obs{2,i}.A = [0; -1];
        tv_obs{2,i}.b = -lane_width/2;
        tv_obs{3,i}.A = [0; 1];
        tv_obs{3,i}.b = -lane_width/2;
    end

    ws_params.name = 'ws_solver_naive';
    ws_params.N = N;
    ws_params.n_x = n_z;
    ws_params.n_u = n_u;
    ws_params.n_obs = n_obs;
    ws_params.n_ineq = n_ineq;
    ws_params.d_ineq = d_ineq;
    ws_params.G = EV_G;
    ws_params.g = EV_g;
    ws_params.optlevel = 3;

    opt_params.name = 'opt_solver_naive';
    opt_params.N = N;
    opt_params.n_x = n_z;
    opt_params.n_u = n_u;
    opt_params.n_obs = n_obs;
    opt_params.n_ineq = n_ineq;
    opt_params.d_ineq = d_ineq;
    opt_params.G = EV_G;
    opt_params.g = EV_g;
    opt_params.d_min = d_min;
    opt_params.Q = Q;
    opt_params.R = R;
    opt_params.R_d = R_d;
    opt_params.z_u = z_u;
    opt_params.z_l = z_l;
    opt_params.u_u = u_u;
    opt_params.u_l = u_l;
    opt_params.du_u = du_u;
    opt_params.du_l = du_l;
    opt_params.dynamics = EV_dynamics;
    opt_params.dt = dt;
    opt_params.optlevel = 3;

    if ~exist('forces_pro_gen_naive', 'dir')
        mkdir('forces_pro_gen_naive')
    end
    cd forces_pro_gen_naive
    obca_controller = obca_controller_FP(true, ws_params, opt_params);
    cd ..
    addpath('forces_pro_gen_naive')

    % Instantiate safety controller
    d_lim = [u_l(1), u_u(1)];
    a_lim = [u_l(2), u_u(2)];
    safety_control = safety_controller(dt, d_lim, a_lim, du_u);
    ebrake_control = ebrake_controller(dt, d_lim, a_lim);

    % Make a copy of EV as the optimal EV, and naive EV
    OEV = EV;

    obca_mpc_safety = false;

    % Data to save
    z_traj = zeros(n_z, T-N+1);
    z_traj(:,1) = EV.traj(:,1);
    u_traj = zeros(n_u, T-N);

    z_preds = zeros(n_z, N+1, T-N);
    u_preds = zeros(n_u, N, T-N);
    z_refs = zeros(n_z, N+1, T-N);

    ws_stats = cell(T-N, 1);
    sol_stats = cell(T-N, 1);

    collide = zeros(T-N, 1);

    FSM_states = cell(T-N, 1);
    strategies = cell(T-N, 1);

    ws_solve_times = zeros(T-N, 1);
    opt_solve_times = zeros(T-N, 1);
    total_times = zeros(T-N, 1);

    exp_params.exp_num = exp_num;
    exp_params.name = 'FP Naive OBCA MPC';
    exp_params.T = T;
    exp_params.T_tv = T_tv;
    exp_params.r = r;
    exp_params.x_max = x_max;
    exp_params.lane_width = lane_width;
    exp_params.strategy_names = strategy_names;
    exp_params.map_dim = map_dim;

    exp_params.controller.N = N;
    exp_params.controller.Q = Q;
    exp_params.controller.R = R;
    exp_params.controller.d_min = d_min;
    exp_params.controller.d_lim = d_lim;
    exp_params.controller.a_lim = a_lim;
    exp_params.dynamics.dt = dt;
    exp_params.dynamics.M = M;
    exp_params.dynamics.L_r = L_r;
    exp_params.dynamics.L_f = L_f;
    exp_params.dynamics.n_z = n_z;
    exp_params.dynamics.n_u = n_u;


    %% 
    for i = 1:T-N
        tIter = tic;
        fprintf('\n=================== Iteration: %i ==================\n', i)
        
        % Get x, y, heading, and velocity from target vehicle over prediction
        % horizon
        TV_x = TV.x(i:i+N);
        TV_y = TV.y(i:i+N);
        TV_th = TV.heading(i:i+N);
        TV_v = TV.v(i:i+N);
        
        TV_pred = [TV_x, TV_y, TV_th, TV_v.*cos(TV_th), TV_v.*sin(TV_th)]';
        
        z_ref = [z_traj(1,i)+[0:N]*dt*v_ref; zeros(1,N+1); zeros(1,N+1); v_ref*ones(1,N+1)]; 
        
        % Generate target vehicle obstacle descriptions
        tv_obs(1,:) = get_car_poly_obs(TV_pred, TV.width, TV.length);

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
        
        % fprintf('------- Solving Naive OBCA MPC -------\n')
        
        % Naive controller
        obca_mpc_ebrake = false;
        obca_mpc_safety = false;
        status_sol = [];
        [status_ws, obca_controller] = obca_controller.solve_ws(z_ws, u_ws, tv_obs);
        if status_ws.success
            ws_solve_times(i) = status_ws.solve_time;
            [z_obca, u_obca, status_sol, obca_controller] = obca_controller.solve(z_traj(:,i), u_prev, z_ref, tv_obs);
        end
        
        if ~status_ws.success || ~status_sol.success
            % If OBCA MPC is infeasible, activate ebrake controller
            obca_mpc_safety = true;
            fprintf('Naive OBCA MPC not feasible, activating safety controller\n')
        else
            opt_solve_times(i) = status_sol.solve_time;
        end
        
        if obca_mpc_safety
            safety_control = safety_control.set_acc_ref(TV_v(1)*cos(TV_th(1)));
            [u_safe, safety_control] = safety_control.solve(z_traj(:,i), TV_pred, u_prev);
            
            z_next = EV_dynamics.f_dt(z_traj(:,i), u_safe);

            actual_collision = check_collision_poly(z_next(1:3), TV_pred(1:3, 2), EV);

            if ~actual_collision
                % Assume safety control is applied for one time step then no
                % control action is applied for rest of horizon
                u_pred = [u_safe zeros(n_u, N-1)];
                z_pred = [z_traj(:,i) zeros(n_z, N)];
                % Simulate this policy
                for j = 1:N
                    z_pred(:,j+1) = EV_dynamics.f_dt(z_pred(:,j), u_pred(:,j));
                end

                fprintf('Applying safety control\n')
            else
                obca_mpc_ebrake = true;
                [u_ebrake, ebrake_control] = ebrake_control.solve(z_traj(:,i), TV_pred, u_prev);
                % Assume ebrake control is applied for one time step then no
                % control action is applied for rest of horizon
                u_pred = [u_ebrake zeros(n_u, N-1)];
                z_pred = [z_traj(:,i) zeros(n_z, N)];
                % Simulate this policy
                for j = 1:N
                    z_pred(:,j+1) = EV_dynamics.f_dt(z_pred(:,j), u_pred(:,j));
                end
                fprintf('Applying ebrake control\n')
            end
        else
            z_pred = z_obca;
            u_pred = u_obca;
        end
        
        % Simulate system forward using first predicted input
        z_traj(:,i+1) = EV_dynamics.f_dt(z_traj(:,i), u_pred(:,1));
        u_traj(:,i) = u_pred(:,1);

        total_times(i) = toc(tIter);
        
        % Save data
        z_preds(:,:,i) = z_pred;
        u_preds(:,:,i) = u_pred;
        
        z_refs(:,:,i) = z_ref;
        
        % Check the collision at the current time step
        collide(i) = check_collision_poly(z_traj(1:3, i), TV_pred(1:3, 1), EV);
        
        strategies{i} = 'N/A';
        if obca_mpc_ebrake
            FSM_states{i} = 'Emergency-Break';
        elseif obca_mpc_safety
            FSM_states{i} = 'Safe-Infeasible';
        else
            FSM_states{i} = 'HOBCA-Unlocked';
        end
        
        ws_stats{i} = status_ws;
        sol_stats{i} = status_sol;

        T_final = i;
        if z_traj(1, i+1) >= x_max
            break
        end
    end

    % Truncate the unused data buffer
    exp_params.T = T_final+N;
    z_traj(:, T_final+2:end) = [];
    u_traj(:, T_final+1:end) = [];

    total_times(T_final+1:end) = [];

    z_preds(:, :, T_final+1:end) = [];
    u_preds(:, :, T_final+1:end) = [];

    z_refs(:, :, T_final+1:end) = [];

    collide(T_final+1:end) = [];

    strategies(T_final+1:end) = [];
    FSM_states(T_final+1:end) = [];

    ws_stats(T_final+1:end) = [];
    sol_stats(T_final+1:end) = [];

    col = any(collide);
    fprintf('Collision: %d \n', col)

    %% Save

    fprintf('\n=================== Complete ==================\n')

    if data_gen
        fpath = sprintf('../datagen/%s_Exp%d_Col%d_%s.mat', filename, exp_num, col, sys_time);
    else
        diary off
        fprintf('Output log saved in: %s_Exp%d_%s.txt\n', filename, exp_num, sys_time);

        fpath = sprintf('../data/%s_Exp%d_Col%d_%s.mat', filename, exp_num, col, sys_time);
    end

    save(fpath, 'exp_params', 'OEV', 'TV', ...
        'z_traj', 'u_traj', 'z_preds', 'u_preds', 'z_refs', ...
        'ws_stats', 'sol_stats', 'strategies', 'FSM_states', ...
        'ws_solve_times', 'opt_solve_times', 'total_times')
    
    fprintf('Data saved in: %s\n', fpath)

    fprintf('Exp_num %d is done. Elapsed time is %d seconds.\n', exp_num, toc(tExp))


end