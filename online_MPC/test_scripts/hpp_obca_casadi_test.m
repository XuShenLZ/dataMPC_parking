clear all
close all

addpath('../')
addpath('../utils')
addpath('../controllers')

%% Load testing data
exp_num = 4;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

%%
n_x = 4;
n_u = 2;

L_r = EV.L/2;
L_f = EV.L/2;
dt = EV.dt;
N = 20;
M = 10;

Q = eye(n_x);
R = eye(n_u);

G = EV.G;
g = EV.g;
d_min = 0.001;

n_obs = 1;
n_ineq = 4;
d_ineq = 2;

dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);

controller = hpp_obca_controller_casadi(N, Q, R, dynamics, G, g, d_min, n_obs, n_ineq, d_ineq);

%%
obs = get_car_poly_obs(zeros(n_x, N), TV.width, TV.length);