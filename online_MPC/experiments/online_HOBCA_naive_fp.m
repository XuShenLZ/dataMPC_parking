clear all
close all
clc

pathsetup();

%% Load testing data
% uiopen('load')
exp_num = 4;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

% Console output saving
if ~isfolder('../data/')
    mkdir('../data')
end

time = datestr(now,'yyyy-mm-dd_HH-MM');
filename = sprintf('FP_NaiveOBCA_Exp%d_%s', exp_num, time);
diary(sprintf('../data/%s.txt', filename))

%%
N = 20; % Prediction horizon
dt = EV.dt; % Time step
T = length(TV.t); % Length of data
v_ref = EV.ref_v; % Reference velocity
y_ref = EV.ref_y; % Reference y
r = sqrt(EV.width^2 + EV.length^2)/2; % Collision buffer radius

n_z = 4;
n_u = 2;

% Instantiate ego vehicle car dynamics
L_r = EV.L/2;
L_f = EV.L/2;
M = 10; % RK4 steps
EV_dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);

% Instantiate obca controller
% Q = diag([0.05 0.1 0.1 0.5]);
% R = diag([0.01 0.01]);
Q = diag([10 1 1 5]);
R = diag([1 1]);

u_u = [0.5; 1.5];
u_l = [-0.5; -1.5];
du_u = [0.6; 5];
du_l = [-0.6; -5];

d_min = 0.01;
% d_min = 0.001;

n_obs = 1;
tv_obs = cell(n_obs, N+1);
lane_width = 8;
% P_u = Polyhedron('V', [-30, 10; 30, 10; -30, lane_width/2; 30, lane_width/2]);
% P_l = Polyhedron('V', [-30, -10; 30, -10; -30, -lane_width/2; 30, -lane_width/2]);
% for i = 1:N+1
%     tv_obs{2,i}.A = P_u.A;
%     tv_obs{2,i}.b = P_u.b;
%     tv_obs{3,i}.A = P_l.A;
%     tv_obs{3,i}.b = P_l.b;
% end

ws_params.name = 'FP_ws_solver_naive';
ws_params.N = N;
ws_params.n_x = n_z;
ws_params.n_u = n_u;
ws_params.n_obs = n_obs;
ws_params.n_ineq = 4;
ws_params.d_ineq = 2;
ws_params.G = EV.G;
ws_params.g = EV.g;

opt_params.name = 'FP_opt_solver_naive';
opt_params.N = N;
opt_params.n_x = n_z;
opt_params.n_u = n_u;
opt_params.n_obs = n_obs;
opt_params.n_ineq = 4;
opt_params.d_ineq = 2;
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

if ~exist('forces_pro_gen', 'dir')
    mkdir('forces_pro_gen')
end
cd forces_pro_gen
obca_controller = obca_controller_FP(true, ws_params, opt_params);
cd ..
addpath('forces_pro_gen')

% Instantiate safety controller
d_lim = [u_l(1), u_u(1)];
a_lim = [u_l(2), u_u(2)];
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
ebrake = zeros(T-N, 1);

exp_params.exp_num = exp_num;
exp_params.name = 'FP Naive OBCA MPC';
exp_params.T = T;
exp_params.lane_width = lane_width;
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

ws_solve_times = zeros(T-N, 1);
opt_solve_times = zeros(T-N, 1);
total_times = zeros(T-N, 1);

%% 
for i = 1:T-N
    tic
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
    tv_obs = get_car_poly_obs(TV_pred, TV.width, TV.length);

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
    
    fprintf('------- Solving Naive OBCA MPC -------\n')
    
    % Naive controller
    obca_mpc_ebrake = false;
    status_sol = [];
    [status_ws, obca_controller] = obca_controller.solve_ws(z_ws, u_ws, tv_obs);
    if status_ws.success
        ws_solve_times(i) = status_ws.solve_time;
        [z_pred, u_pred, status_sol, obca_controller] = obca_controller.solve(z_traj(:,i), u_prev, z_ref, tv_obs);
    end
    
    if ~status_ws.success || ~status_sol.success
        % If OBCA MPC is infeasible, activate ebrake controller
        obca_mpc_ebrake = true;
        fprintf('Naive OBCA MPC not feasible, activating emergency brake\n')
    else
        opt_solve_times(i) = status_sol.solve_time;
    end
    
    if obca_mpc_ebrake
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
    
    % Simulate system forward using first predicted input
    z_traj(:,i+1) = EV_dynamics.f_dt(z_traj(:,i), u_pred(:,1));
    u_traj(:,i) = u_pred(:,1);
    
    total_times(i) = toc;
    
    % Save data
    z_preds(:,:,i) = z_pred;
    u_preds(:,:,i) = u_pred;
    
    z_refs(:,:,i) = z_ref;

    % Check the collision at the current time step
    collide(i) = check_current_collision(z_traj, EV, TV, i);
    ebrake(i) = obca_mpc_ebrake;

    ws_stats{i} = status_ws;
    sol_stats{i} = status_sol;
end

fprintf('\n=================== Complete ==================\n')
fprintf('Output log saved in: %s.txt, data saved in: %s.mat\n', filename, filename)

save(sprintf('../data/%s.mat', filename), 'exp_params', 'OEV', 'TV', ...
    'z_traj', 'u_traj', 'z_preds', 'u_preds', 'z_refs', 'ws_stats', 'sol_stats', 'collide', 'ebrake', ...
    'ws_solve_times', 'opt_solve_times', 'total_times')

diary off