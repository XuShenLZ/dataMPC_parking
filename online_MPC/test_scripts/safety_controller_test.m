close all
clear all

addpath('../../nominal_MPC')
addpath('../')

wid = 2.0607;
len = 4.9047;
r = sqrt(wid^2 + len^2)/2;

dt = 0.1;
d_lim = [-0.35, 0.35];
a_lim = [-1, 1];
safety_control = safety_controller(dt, d_lim, a_lim);
control = safety_controller(dt, d_lim, a_lim);
control = control.set_speed_ref(2);

x_0 = [-5; 2; 0; 2];

TV_0 = [35; 2; pi; 3];

x_traj = x_0;
u_traj = [];

TV_traj = TV_0;

u = zeros(2,1);
brake = false;

for k = 1:200
    EV_k = x_traj(:,k);
    TV_k = TV_traj(:,k);
    
    EV_x = EV_k(1);
    EV_y = EV_k(2);
    EV_th = EV_k(3);
    EV_v = EV_k(4);

    TV_x = TV_k(1);
    TV_y = TV_k(2);
    TV_th = TV_k(3);
    TV_v = TV_k(4);

    rel_x = TV_x - EV_x;
    rel_vx = TV_v*cos(TV_th) - EV_v*cos(EV_th);
    d = norm(TV_k(1:2) - EV_k(1:2), 2);

    min_ts = ceil(abs(rel_vx)/abs(a_lim(1))/dt);
    v_brake = abs(rel_vx)-[0:min_ts]*dt*abs(a_lim(1));
%     brake_dist = sum(abs(v_brake)*dt) + abs(TV_v*cos(TV_th))*(min_ts+1)*dt + 2*r;
    brake_dist = sum(abs(v_brake)*dt) + 4*r;

    if ~brake && d <= brake_dist
        brake = true;
    end
    
    if brake
        safety_control = safety_control.set_speed_ref(TV_v*cos(TV_th));
    end
    
    if brake
        [u, safety_control] = safety_control.solve(EV_k, TV_k, u);
    else
        [u, control] = control.solve(EV_k, TV_k, u);
    end
            
    x_kp1 = bikeFE_CoG(EV_k, u, dt);
    x_traj = [x_traj x_kp1];
    u_traj = [u_traj u];
    
    TV_x = TV_k(1);
    TV_u = zeros(2,1);
    if TV_x < 3
        TV_u = [0.35; 0];
    end
        
    TV_kp1 = bikeFE_CoG(TV_k, TV_u, dt);
    TV_traj = [TV_traj TV_kp1];
end

T = size(u_traj, 2);
map_dim = [-15 30 -5 5];

plt_opts.circle = false;
plt_opts.frame = true;
plt_opts.color = 'b';
plt_opts.alpha = 0.5;

TV_plt_opts.circle = false;
TV_plt_opts.frame = false;
TV_plt_opts.color = 'y';
TV_plt_opts.alpha = 0.5;

p1 = [];
l1 = [];
p2 = [];
l2 = [];

% x_f = trajectory_data(rollout).state(:,end);

fig = figure('Position', [50 50 1200 600]);

ax1 = axes('Position',[0.05 0.3 0.4 0.4]);
% yline(3.5, '-.', 'color', '#7E2F8E', 'linewidth', 2)
% hold on
% yline(-3.5, '-.', 'color', '#7E2F8E', 'linewidth', 2)
axis equal
axis(map_dim);

ax_th = axes('Position',[0.5 0.75 0.45 0.2]);
th_l = animatedline(ax_th, 'color', '#0072BD', 'linewidth', 2);
ylabel('theta')
axis([1 T -pi pi])

ax_v = axes('Position',[0.5 0.52 0.45 0.2]);
v_l = animatedline(ax_v, 'color', '#0072BD', 'linewidth', 2);
ylabel('v')
axis([1 T -3 3])

ax_d = axes('Position',[0.5 0.28 0.45 0.2]);
d_l = animatedline(ax_d, 'color', '#0072BD', 'linewidth', 2);
ylabel('delta')
axis([1 T -0.35 0.35])

ax_a = axes('Position',[0.5 0.05 0.45 0.2]);
a_l = animatedline(ax_a, 'color', '#0072BD', 'linewidth', 2);
ylabel('a')
axis([1 T -1 1])

F(T) = struct('cdata',[],'colormap',[]);

for i = 1:T
    delete(p1)
    delete(l1)
    delete(p2)
    delete(l2)
    
    EV_k = x_traj(1,i);
    y_k = x_traj(2,i);
    EV_theta = x_traj(3,i);
    EV_v = x_traj(4,i);
    
    delta_k = u_traj(1,i);
    a_k = u_traj(2,i);
    
    TV_x_k = TV_traj(1,i);
    TV_y_k = TV_traj(2,i);
    TV_theta_k = TV_traj(3,i);
    
    axes(ax1)
    [p1, l1] = plotCar(EV_k, y_k, EV_theta, wid, len, plt_opts);
    hold on
    [p2, l2] = plotCar(TV_x_k, TV_y_k, TV_theta_k, wid, len, TV_plt_opts);

    axis equal
    axis(map_dim);
    
    addpoints(th_l, i, EV_theta);
    addpoints(v_l, i, EV_v);
    addpoints(d_l, i, delta_k);
    addpoints(a_l, i, a_k);
    drawnow
    
    pause(0.05)
    
    F(i) = getframe(fig);
end

movie_name = 'safety_controller.mp4';
[file,path] = uiputfile(movie_name);

v = VideoWriter([path, file], 'MPEG-4');
v.FrameRate = 10;
open(v);
writeVideo(v,F);
close(v);

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
	lf = 2.9428/2;
	lr = 2.9428/2;

	zp = z;

	% psi_new = z(3) + dt/2 * z(4) / lr * sin(u(1));
	v_new = z(4) + dt/2 * u(2);

	zp(1) = z(1) + dt * v_new * cos(z(3) + u(1));
	zp(2) = z(2) + dt * v_new * sin(z(3) + u(1));
	zp(3) = z(3) + dt * v_new / lr * sin(u(1));
	zp(4) = z(4) + dt * u(2);
end