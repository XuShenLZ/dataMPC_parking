close all
clear all

addpath('../controllers')
addpath('../dynamics')

% global n_x n_u n_obs n_ineq d_ineq G g 

import casadi.*

%% Load testing data
exp_num = 4;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

n_x = 4;
n_u = 2;

L_r = EV.L/2;
L_f = EV.L/2;
dt = EV.dt;
N = 5;
M = 10;

Q = eye(n_x);
R = eye(n_u);

u_u = [0.35; 1];
u_l = [-0.35; -1];
du_u = [0.3; 3];
du_l = [-0.3; -3];

G = EV.G;
g = EV.g;
d_min = 0.001;

n_obs = 1;
n_ineq = 4;
d_ineq = 2;

dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);

ws_params.name = 'fp_ws_solver';
ws_params.N = N;
ws_params.n_x = n_x;
ws_params.n_u = n_u;
ws_params.n_obs = n_obs;
ws_params.n_ineq = n_ineq;
ws_params.d_ineq = d_ineq;
ws_params.G = G;
ws_params.g = g;

opt_params.name = 'fp_opt_solver';
opt_params.N = N;
opt_params.n_x = n_x;
opt_params.n_u = n_u;
opt_params.n_obs = n_obs;
opt_params.n_ineq = n_ineq;
opt_params.d_ineq = d_ineq;
opt_params.G = G;
opt_params.g = g;
opt_params.d_min = d_min;
opt_params.Q = Q;
opt_params.R = R;
opt_params.u_u = u_u;
opt_params.u_l = u_l;
opt_params.du_u = du_u;
opt_params.du_l = du_l;
opt_params.dynamics = dynamics;
opt_params.dt = dt;

generate_forces_pro_ws_solver(ws_params);
generate_forces_pro_opt_solver(opt_params);
