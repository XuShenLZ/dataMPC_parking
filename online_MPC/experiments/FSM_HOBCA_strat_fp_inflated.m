%% FSM_HOBCA_strat_fp: Finite State Machine based strat HOBCA
% exp_num (int, 1-486): The exp_num going to be evaluated
% data_gen (boolean): whether this is doing datagen or not
function [col, T_final] = FSM_HOBCA_strat_fp_inflated(exp_num, data_gen)
    
    tExp = tic;

    sys_time = datestr(now,'yyyy-mm-dd_HH-MM');
    filename = 'FSM_HOBCA_strat_fp';

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

    %% Construct strategy prediction model
    model_name = 'nn_strategy_TF-trainscg_h-40_AC-tansig_ep2000_CE0.17453_2020-08-04_15-42';
    model_file = strcat('../../models/', model_name, '.mat');

    V = 0.01 * eye(3);
    W = 0.5 * eye(3);
    Pm = 0.2 * eye(3);
    predictor = StrategyPredictor(model_file, V, W, Pm);

    %% Experiment parameters
    N = 20; % Prediction horizon
    dt = EV.dt; % Time step
    T = 1500; % Max time
    T_tv = length(TV.t); % Length of TV data
    x_max = 30; % The right most x coordinate
    v_ref = EV.ref_v; % Reference velocity
    y_ref = EV.ref_y; % Reference y
    r = sqrt(EV.width^2 + EV.length^2)/2; % Collision buffer radius
    confidence_thresh = 0.55;
    lock_steps = 20;

    strategy_names = ["Left", "Right", "Yield"];

    % Extend TV traj to remain staionary after parked
    TV.x(end+1:T) = TV.x(end);
    TV.y(end+1:T) = TV.y(end);
    TV.heading(end+1:T) = TV.heading(end);
    TV.v(end+1:T) = TV.v(end);

    % Make a copy of EV as the optimal EV, and naive EV
    OEV = EV;

    %% ================================
    % ======= Solver Specific Code

    n_z = 4;
    n_u = 2;

    % Instantiate ego vehicle car dynamics
    L_r = EV.L/2;
    L_f = EV.L/2;
    M = 10; % RK4 steps
    EV_dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);

    % Instantiate obca controller
    % Q = diag([0.05 0.1 0.1 0.5]);
    Q = diag([10 1 1 5]);
    R = diag([1 1]);

    u_u = [0.5; 1.5];
    u_l = [-0.5; -2.5];
    du_u = [0.6; 5];
    du_l = [-0.6; -8];

    d_min = 0.01; %0.001;

    % n_obs = 1;
    % n_ineq = [4];
    n_obs = 3;
    n_ineq = [4,1,1];
    d_ineq = 2;

    tv_obs = cell(n_obs, N+1);
    lane_width = 8;
    for i = 1:N+1
        tv_obs{2,i}.A = [0, -1];
        tv_obs{2,i}.b = -lane_width/2;
        tv_obs{3,i}.A = [0, 1];
        tv_obs{3,i}.b = -lane_width/2;
    end

    ws_params.name = 'FP_ws_solver_strat';
    ws_params.N = N;
    ws_params.n_x = n_z;
    ws_params.n_u = n_u;
    ws_params.n_obs = n_obs;
    ws_params.n_ineq = n_ineq;
    ws_params.d_ineq = d_ineq;
    ws_params.G = EV.G;
    ws_params.g = EV.g;
    ws_params.optlevel = 3;

    opt_params.name = 'FP_opt_solver_strat';
    opt_params.N = N;
    opt_params.n_x = n_z;
    opt_params.n_u = n_u;
    opt_params.n_obs = n_obs;
    opt_params.n_ineq = n_ineq;
    opt_params.d_ineq = d_ineq;
    opt_params.G = EV.G;
    opt_params.g = EV.g;
    opt_params.d_min = d_min;
    opt_params.Q = Q;
    opt_params.R = R;
    opt_params.u_u = u_u;
    opt_params.u_l = u_l;
    opt_params.du_u = du_u;
    opt_params.du_l = du_l;
    opt_params.dynamics = EV_dynamics;
    opt_params.dt = dt;
    opt_params.optlevel = 3;

    if ~exist('forces_pro_gen', 'dir')
        mkdir('forces_pro_gen')
    end
    cd forces_pro_gen
    obca_controller = hpp_obca_controller_FP(false, ws_params, opt_params);
    cd ..
    addpath('forces_pro_gen')

    % Instantiate safety controller
    d_lim = [u_l(1), u_u(1)];
    a_lim = [u_l(2), u_u(2)];
    safety_control = safety_controller(dt, d_lim, a_lim, du_u);
    % safety_control = safety_controller_v2(dt, d_lim, a_lim, du_u);
    ebrake_control = ebrake_controller(dt, d_lim, a_lim);


    %% =====================
    % ===== Data to save
    z_traj = zeros(n_z, T-N+1);
    z_traj(:,1) = EV.traj(:,1);
    u_traj = zeros(n_u, T-N);

    z_preds = zeros(n_z, N+1, T-N);
    u_preds = zeros(n_u, N, T-N);
    z_refs = zeros(n_z, N+1, T-N);

    hyps = cell(T-N, 1);

    collide = zeros(T-N, 1);
    scores = zeros(3, T-N);

    FSM_states = cell(T-N, 1);
    strategies = cell(T-N, 1);

    ws_stats = cell(T-N, 1);
    sol_stats = cell(T-N, 1);

    ws_solve_times = zeros(T-N, 1);
    opt_solve_times = zeros(T-N, 1);
    total_times = zeros(T-N, 1);
    
    vel_dirs = zeros(1, N+1, T-N);
    scalings = zeros(1, N+1, T-N);

    exp_params.exp_num = exp_num;
    exp_params.name = 'FP Strategy OBCA MPC - FSM';
    exp_params.T = T;
    exp_params.T_tv = T_tv;
    exp_params.r = r;
    exp_params.x_max = x_max;
    exp_params.lane_width = lane_width;
    exp_params.model = model_name;
    exp_params.confidence_thresh = confidence_thresh;
    exp_params.lock_steps = lock_steps;
    exp_params.strategy_names = strategy_names;

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
    exp_params.filter.V = V;
    exp_params.filter.W = W;
    exp_params.filter.Pm = Pm;

    % Initialize Finite State Machine
    FSM = FiniteStateMachine(exp_params);

    %% Experiment Loop
    exp_states.t = 0;
    exp_states.EV_curr = zeros(5, 1);
    exp_states.TV_pred = zeros(5, N+1);
    exp_states.score = ones(3, 1) / 3;
    exp_states.feas = true;
    exp_states.actual_collision = false;
    exp_states.ref_col = zeros(1, N+1);

    i = 0;
    while ~isequal(FSM.state, "End")
        tIter = tic;
        i = i+1;
        fprintf('\n=================== Iteration: %i ==================\n', i)
        
        %% ==========================
        % Get x, y, heading, and velocity from ego vehicle at current timestep
        EV_x  = z_traj(1,i);
        EV_y  = z_traj(2,i);
        EV_th = z_traj(3,i);
        EV_v  = z_traj(4,i);

        EV_curr = [EV_x; EV_y; EV_th; EV_v*cos(EV_th); EV_v*sin(EV_th)];
        
        % Get x, y, heading, and velocity from target vehicle over prediction
        % horizon
        TV_x = TV.x(i:i+N);
        TV_y = TV.y(i:i+N);
        TV_th = TV.heading(i:i+N);
        TV_v = TV.v(i:i+N);
        
        TV_pred = [TV_x, TV_y, TV_th, TV_v.*cos(TV_th), TV_v.*sin(TV_th)]';

        % Generate target vehicle obstacle descriptions
        tv_obs(1,:) = get_car_poly_obs(TV_pred, TV.width, TV.length);
        
        % Get target vehicle trajectory relative to ego vehicle state
        if any(FSM.state == ["Safe-Confidence", "Safe-Yield", "Safe-Infeasible"]) || EV_v*cos(EV_th) <= 0
            tmp = [EV_x; EV_y; EV_th; 0; EV_v*sin(EV_th)];
            rel_state = TV_pred - tmp;
        else
            rel_state = TV_pred - EV_curr;
        end
        
        %% ========================
        % Predict strategy to use based on relative prediction of target
        % vehicle
        score = predictor.predict(rel_state);
        fprintf('Confidence: %g \n', max(score));

        %% =======================
        % Make a state transition using current status
        exp_states.t = i;
        exp_states.EV_curr = EV_curr;
        exp_states.TV_pred = TV_pred;
        exp_states.score = score;
        strategy = FSM.state_transition(exp_states);

        % If the task is complete
        if FSM.state == "End"
            break
        end

        %% ========================
        % Generate reference trajectory
        if any(FSM.state == ["HOBCA-Unlocked", "HOBCA-Locked"])
            % If strategy is HOBCA (lock/unlock), discount reference velocity based on max
            % likelihood
            EV_x_ref = EV_x + [0:N]*dt*v_ref;
%             EV_v_ref = max(score)*v_ref*ones(1, N+1);
            EV_v_ref = v_ref*ones(1, N+1);
        elseif any(FSM.state == ["Free-Driving", "Safe-Confidence", "Safe-Yield", "Safe-Infeasible", "Emergency-Break"])
            % If stratygy is Safe_Confidence, Safe-Yield, Safe_Infeasible, EB
            EV_x_ref = EV_x + [0:N]*dt*v_ref;
            EV_v_ref = v_ref*ones(1, N+1);
        else
            error('Unrecognized state when generating reference trajectory.')
        end
        
        EV_y_ref = zeros(1, length(EV_x_ref));
        EV_h_ref = zeros(1, length(EV_x_ref));
        z_ref = [EV_x_ref; EV_y_ref; EV_h_ref; EV_v_ref];
        
        % Check which points along the reference trajectory would result in
        % collision. Collision is defined as the reference point at step k 
        % being contained in O_TV(k) + B(r), where O_TV(k) is the region
        % occupied by the target vehicle at step k along the prediction horizon
        % and B(r) is the 2D ball with radius equal to the collision buffer
        % radius of the ego vehicle
        
        % ======= Use the ref traj to detect collision
        z_detect = z_ref; % Use the ref to construct hpp
        ref_col = [];
        
        scale_mult = 1;
        for j = 1:N+1
            scaling = max(1, scale_mult*abs(TV_v(j))/v_ref);
            vel_dir = sign(TV_v(j));
            
            scalings(1,j,i) = scaling;
            vel_dirs(1,j,i) = vel_dir;
            
            collision = check_collision_point_inflated(z_detect(1:2,j), TV_x(j), TV_y(j), TV_th(j), TV.width, TV.length, r, vel_dir, scaling);
            ref_col = [ref_col, collision];
        end

        % Test the state transition again and see whether the HOBCA will be locked
        exp_states.ref_col = ref_col;
        strategy = FSM.state_transition(exp_states);
        
        % Generate hyperplane constraints
        hyp = cell(N+1,1);
        for j = 1:N+1
            scaling = max(1, scale_mult*abs(TV_v(j))/v_ref);
            vel_dir = sign(TV_v(j));
        
            ref = z_detect(1:2, j);
            collision = ref_col(j);
            if collision
                if strategy == "Left"
                    dir = [0; 1];
                elseif strategy == "Right"
                    dir = [0; -1];
                else
                    dir = [EV_x-TV_x(j); EV_y-TV_y(j)];
                    dir = dir/(norm(dir));
                end
                % ==== Unbiased Tight HPP
                [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp_tight_inflated(ref, dir, TV_x(j), TV_y(j), TV_th(j), ...
                    TV.width, TV.length, r, vel_dir, scaling);

                % =====
                hyp{j}.w = [hyp_w; zeros(n_z-length(hyp_w),1)];
                hyp{j}.b = hyp_b;
                hyp{j}.pos = hyp_xy;
            else
                % Placeholder hyperplane constraint which will always be
                % satisfied
                hyp{j}.w = [sign(ref(1)); sign(ref(2)); zeros(2,1)];
                hyp{j}.b = 0;
                hyp{j}.pos = nan;
            end
        end

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
        
        fprintf('------- Solving Strategy OBCA -------\n')
        
        status_sol = [];
        [status_ws, obca_controller] = obca_controller.solve_ws(z_ws, u_ws, tv_obs);
        if status_ws.success
            ws_solve_times(i) = status_ws.solve_time;
            [z_obca, u_obca, status_sol, obca_controller] = obca_controller.solve(z_traj(:,i), u_prev, z_ref, tv_obs, hyp);
        end

        if status_ws.success && status_sol.success
            feas = true;
            
            actual_collision = false;
            exp_states.actual_collision = actual_collision;

            opt_solve_times(i) = status_sol.solve_time;
        else
            feas = false;
        end
        
        % Test the state transition again to use the feasiblity of HOBCA
        exp_states.feas = feas;
        strategy = FSM.state_transition(exp_states);

        if any(FSM.state == ["Safe-Confidence", "Safe-Yield", "Safe-Infeasible"])

            % For safety controller v1
            safety_control = safety_control.set_acc_ref(TV_v(1)*cos(TV_th(1)));

            % For safety controller v2
            % safety_control = safety_control.set_acc_ref(TV_x(1), TV_v(1)*cos(TV_th(1)));
            [u_safe, safety_control] = safety_control.solve(z_traj(:,i), TV_pred, u_prev);
            
            z_next = EV_dynamics.f_dt(z_traj(:,i), u_safe);

            actual_collision = check_collision_poly(z_next(1:3), TV_pred(1:3, 2), EV);
            exp_states.actual_collision = actual_collision;
        end

        % Test the state transition again to see whether EB is needed using actual collision
        strategy = FSM.state_transition(exp_states);

        if any(FSM.state == ["Free-Driving", "HOBCA-Unlocked", "HOBCA-Locked"])
            z_pred = z_obca;
            u_pred = u_obca;
            fprintf('Applying HOBCA control\n')
        elseif any(FSM.state == ["Safe-Confidence", "Safe-Yield", "Safe-Infeasible"])
            % Assume safety control is applied for one time step then no
            % control action is applied for rest of horizon
            u_pred = [u_safe zeros(n_u, N-1)];
            z_pred = [z_traj(:,i) zeros(n_z, N)];
            % Simulate this policy
            for j = 1:N
                z_pred(:,j+1) = EV_dynamics.f_dt(z_pred(:,j), u_pred(:,j));
            end

            fprintf('Applying safety control\n')
        elseif FSM.state == "Emergency-Break"
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
        else
            error('Unrecognized State when applying control');
        end
        
        % Simulate system forward using first predicted input
        z_traj(:,i+1) = EV_dynamics.f_dt(z_traj(:,i), u_pred(:,1));
        u_traj(:,i) = u_pred(:,1);

        
        % Save data
        z_preds(:,:,i) = z_pred;
        u_preds(:,:,i) = u_pred;
        
        z_refs(:,:,i) = z_ref;
        hyps{i} = hyp;

        % Check the collision at the current time step
        collide(i) = check_collision_poly(z_traj(1:3, i), TV_pred(1:3, 1), EV);
        
        scores(:,i) = score;
        strategies{i} = strategy;
        FSM_states{i} = FSM.state;
        
        ws_stats{i} = status_ws;
        sol_stats{i} = status_sol;

        total_times(i) = toc(tIter);

        T_final = i;
    end

    %% Truncate the unused data buffer
    exp_params.T = T_final+N;
    z_traj(:, T_final+2:end) = [];
    u_traj(:, T_final+1:end) = [];

    z_preds(:, :, T_final+1:end) = [];
    u_preds(:, :, T_final+1:end) = [];

    z_refs(:, :, T_final+1:end) = [];
    hyps(T_final+1:end) = [];

    collide(T_final+1:end) = [];

    scores(:, T_final+1:end) = [];
    strategies(T_final+1:end) = [];
    FSM_states(T_final+1:end) = [];

    ws_stats(T_final+1:end) = [];
    sol_stats(T_final+1:end) = [];

    total_times(T_final+1:end) = [];

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
        'z_traj', 'u_traj', 'z_preds', 'u_preds', 'z_refs', 'hyps', ...
        'collide', 'scores', 'strategies', 'FSM_states', ...
        'ws_stats', 'sol_stats', ...
        'ws_solve_times', 'opt_solve_times', 'total_times', ...
        'vel_dirs', 'scalings')

    fprintf('Data saved in: %s\n', fpath)

    fprintf('Exp_num %d is done. Elapsed time is %d seconds.\n', exp_num, toc(tExp))
end