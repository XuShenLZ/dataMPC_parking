close all
clear all

addpath('../nominal_MPC')

dt = 0.1;
d_lim = [-0.35, 0.35];
a_lim = [-1, 1];
safety_control = safety_controller(dt, d_lim, a_lim);

x_0 = [0; 0.5; 0; 0];

x_traj = x_0;
u_traj = [];

for k = 1:300
%     x_k = [x_traj(1:3,k); x_traj(5,k); x_traj(4,k)];
    x_k = x_traj(:,k);
    if k < 100
        u = [0; 0.1];
    else
        [u, safety_control] = safety_control.solve(x_k, zeros(4,1), u);
    end
    
    x_kp1 = bikeFE_CoG(x_traj(:,k), u, dt);
    x_traj = [x_traj x_kp1];
    u_traj = [u_traj u];
end

T = size(u_traj, 2);
map_dim = [-1 7 -1 1];

plt_opts.circle = false;
plt_opts.frame = true;
plt_opts.color = 'b';
plt_opts.alpha = 0.5;
wid = 0.3;
len = 0.7;

p = [];
l = [];

% x_f = trajectory_data(rollout).state(:,end);

fig_1 = figure('Position', [50 50 600 600]);
% plot_car(x_f(1), x_f(2), x_f(3), wid, len, plt_opts);

fig_2 = figure('Position', [700 50 600 600]);

ax_1 = subplot(4,1,1);
d_l = animatedline(ax_1, 'color', '#0072BD', 'linewidth', 2);
ylabel('theta')
d_lim = [1 T -pi pi];
axis(d_lim)

ax_2 = subplot(4,1,2);
v_l = animatedline(ax_2, 'color', '#0072BD', 'linewidth', 2);
ylabel('v')
v_lim = [1 T -2 2];
axis(v_lim)

ax_3 = subplot(4,1,3);
o_l = animatedline(ax_3, 'color', '#0072BD', 'linewidth', 2);
ylabel('delta')
omega_lim = [1 T -0.35 0.35];
axis(omega_lim)

ax_4 = subplot(4,1,4);
a_l = animatedline(ax_4, 'color', '#0072BD', 'linewidth', 2);
ylabel('a')
a_lim = [1 T -1 1];
axis(a_lim)

for i = 1:T
    delete(p)
    delete(l)
    
    x_k = x_traj(1,i);
    y_k = x_traj(2,i);
    theta_k = x_traj(3,i);
    v_k = x_traj(4,i);
    
    delta_k = u_traj(1,i);
    a_k = u_traj(2,i);
    
    set(0, 'currentfigure', fig_1)
    [p, l] = plotCar(x_k, y_k, theta_k, wid, len, plt_opts);
    hold on
    
    axis equal
    axis(map_dim);
    
    addpoints(d_l, i, theta_k);
    addpoints(v_l, i, v_k);
    addpoints(o_l, i, delta_k);
    addpoints(a_l, i, a_k);
    drawnow
    
    pause(0.05)
end

function x_kp1 = f_true(x_k, u_k, dt)
    L_r = 0.3;
    L_f = 0.3;
    x_kp1 = zeros(5,1);
    beta = @(d) atan2(L_r*tan(d), L_r+L_f);
    
    x_kp1(1) = x_k(1) + dt*x_k(5)*cos(x_k(3)+beta(x_k(4)));
    x_kp1(2) = x_k(2) + dt*x_k(5)*sin(x_k(3)+beta(x_k(4)));
    x_kp1(3) = x_k(3) + dt*x_k(5)*sin(beta(x_k(4)))/L_r;
    x_kp1(4) = x_k(4) + dt*u_k(1);
    x_kp1(5) = x_k(5) + dt*u_k(2);
end

function zp = bikeFE_CoG(z, u, dt)

	% ======== CoG Reference
	lf = 0.3;
	lr = 0.3;

	zp = z;

	% psi_new = z(3) + dt/2 * z(4) / lr * sin(u(1));
	v_new = z(4) + dt/2 * u(2);

	zp(1) = z(1) + dt * v_new * cos(z(3) + u(1));
	zp(2) = z(2) + dt * v_new * sin(z(3) + u(1));
	zp(3) = z(3) + dt * v_new / lr * sin(u(1));
	zp(4) = z(4) + dt * u(2);
end