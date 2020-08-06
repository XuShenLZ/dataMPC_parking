close all
clear all

addpath('../nominal_MPC')

%% Load testing data
% uiopen('load')
exp_num = 4;
exp_file = strcat('../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

%%
N = 20; % Prediction horizon
dt = EV.dt; % Time step
T = length(TV.t); % Length of data
v_ref = EV.ref_v; % Reference velocity
y_ref = EV.ref_y; % Reference y
r = sqrt(EV.width^2 + EV.length^2)/2; % Collision buffer radius

EV_plt_opts.circle = true;
EV_plt_opts.color = 'b';
EV_plt_opts.alpha = 0.5;

TV_plt_opts.circle = false;
TV_plt_opts.color = 'r';
TV_plt_opts.alpha = 0.5;

map_dim = [-30 30 -10 10];
p_EV = [];
c_EV = [];
p_TV = [];
t_EV_ref = [];

EV_x = 6;
EV_y = 5;

dir_global = [0; 1];

TV_x = 5;
TV_y = 3;
TV_theta = 7*pi/6;

R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
collision = check_collision([EV_x; EV_y], TV_x, TV_y, TV_theta, TV.width, TV.length, r);

phi = linspace(0, 2*pi, 200);
x = [];
y = [];
for i = 1:length(phi)
    [x_b, y_b, ~, ~] = get_collision_boundary_point(0, 0, phi(i), TV.width, TV.length, r);
    x = [x x_b];
    y = [y y_b];
end

coll_bound_global = R(TV_theta)*[x; y] + [TV_x; TV_y];

[hyp_xy, w_global, b_global] = get_extreme_pt_hyp([EV_x; EV_y], ...
    dir_global, TV_x, TV_y, TV_theta, TV.width, TV.length, r);

if w_global(2) == 0
    hyp_x = [b_global, b_global];
    hyp_y = [TV_y-TV.width, TV_y+TV.width];
else
    hyp_x = [TV_x-2*TV.length, TV_x+2*TV.length];
    hyp_y = (-w_global(1)*hyp_x+b_global)/w_global(2);
end

figure()
plot(coll_bound_global(1,:), coll_bound_global(2,:))
hold on
[p, l] = plotCar(TV_x, TV_y, TV_theta, TV.width, TV.length, TV_plt_opts);
plot(EV_x, EV_y, 'go')
plot([EV_x EV_x+dir_global(1)], [EV_y EV_y+dir_global(2)], 'g')
plot(hyp_x, hyp_y)
plot(hyp_xy(1), hyp_xy(2), 'go')

xlim([-10, 10])
ylim([-10, 10])
axis equal

%%
x_o = -4;
y_o = -2.5;

phi = linspace(0, 2*pi, 200);
x = [];
y = [];
for i = 1:length(phi)
    [x_b, y_b, ~, ~] = get_collision_boundary_point(0, 0, phi(i), TV.width, TV.length, r);
    x = [x x_b];
    y = [y y_b];
end

figure()
plot(x, y)
hold on
plot(x_o, y_o, 'go')
plot(0, 0, 'ro')
axis equal