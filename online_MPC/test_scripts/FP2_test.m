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

% controller = hpp_obca_controller_FP(N, Q, R, dynamics, G, g, d_min, n_obs, n_ineq, d_ineq);

% function ws_obj = eval_ws_obj(z)
%     global n_obs n_ineq
%     
%     d = z(n_obs*n_ineq + n_obs*n_ineq+1:end);
%     ws_obj = -sum(d);
% end
%     
% function ws_eq = eval_ws_eq(z, p)
%     global n_x n_u n_obs n_ineq d_ineq G g 
%     
%     t_ws = p(1:2);
%     R_ws = [cos(p(3)), -sin(p(3)); sin(p(3)), cos(p(3))];
% 
%     ws_eq = [];
%     for i = 1:n_obs
%         A = reshape(p(n_x+n_u+(i-1)*n_ineq*d_ineq+1:n_x+n_u+i*n_ineq*d_ineq), n_ineq, d_ineq);
%         b = p(n_x+n_u+n_obs*n_ineq*d_ineq+(i-1)*n_ineq+1:n_x+n_u+n_obs*n_ineq*d_ineq+i*n_ineq);
%         lambda = z((i-1)*n_ineq+1:i*n_ineq);
%         mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);
%         d = z(n_obs*n_ineq+n_obs*n_ineq+i);
%         ws_eq = vertcat(ws_eq, -dot(g, mu)+dot(mtimes(A, t_ws)-b, lambda)-d);
%         ws_eq = vertcat(ws_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_ws)), lambda));
%     end
% end
% 
% function ws_ineq = eval_ws_ineq(z, p)
%     global n_x n_u n_obs n_ineq d_ineq
%     
%     ws_ineq = [];
%     for i = 1:n_obs
%         A = reshape(p(n_x+n_u+(i-1)*n_ineq*d_ineq+1:n_x+n_u+i*n_ineq*d_ineq), n_ineq, d_ineq);
% %                     b = p(self.n_x+self.n_u+self.n_obs*self.n_ineq*self.d_ineq+(i-1)*self.n_ineq+1:self.n_x+self.n_u+self.n_obs*self.n_ineq*self.d_ineq+i*self.n_ineq);
%         lambda = z((i-1)*n_ineq+1:i*n_ineq);
%         mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);
%         ws_ineq = vertcat(ws_ineq, dot(mtimes(transpose(A), lambda), mtimes(transpose(A),lambda)));
%         ws_ineq = vertcat(ws_ineq, lambda);
%         ws_ineq = vertcat(ws_ineq, mu);
%     end
% end