%% Predict the learned hyperplane using network
% and plot it with the optimal one
clear('all');
close('all');
clc

%% Load Dataset and network
[file, path] = uigetfile('.mat', 'Select Dataset');
load([path, file])
[file, path] = uigetfile('.mat', 'Select Network');
load([path, file])

%% Load TV data
exp_num = 1;
load(['../data/exp_num_', num2str(exp_num), '.mat'])

training_data = training_set{exp_num};
T = length(training_data);
N = 20;

%% Predict the hyperplane using trained function
for t = 1:T-N
	[feature, ~, ego_ref] = gen_feature_label(training_data, t, N);

	feature_flat = reshape(feature, [], 1);

	% Predict
	Y = net(feature_flat);

	% Return to shape [w1;w2;b] x timesteps
	local_hpp = reshape(Y, [], N+1);

	% Transform back to global frame
	global_hpp = local_hpp;
	for k = 0:N
		global_hpp(3, k+1) = local_hpp(3, k+1) + local_hpp(1:2, k+1)'*ego_ref;
	end

	predicted_data(t).hyperplane.w = global_hpp(1:2, 1);
	predicted_data(t).hyperplane.b = global_hpp(3, 1);
end

%% Plot
fig = figure();
map_dim = [-30 30 -10 10];
plot(TV.x, TV.y, 'k--');
hold on

for i = 1:T-N
    w = training_data(i).hyperplane.w;
    b = training_data(i).hyperplane.b;

    hyp_y = [-10, 10];
    hyp_x = -(w(2)*hyp_y+b)/w(1);

    p_hyp = plot(hyp_x, hyp_y, 'k');
    hold on

    w_hat = predicted_data(i).hyperplane.w;
    b_hat = predicted_data(i).hyperplane.b;

    hyp_y_hat = [-10, 10];
    hyp_x_hat = -(w_hat(2)*hyp_y_hat+b_hat)/w_hat(1);

    p_hyp_hat = plot(hyp_x_hat, hyp_y_hat, 'g--');
    hold on
    
    % === Uncomment this part to plot the vehicle shape at current time step
    % plt_ops.alpha = 0.5;
    % plt_ops.circle = false;
    % plt_ops.color = 'blue';
    % [p_EV, c_EV] = plotCar(training_data(i).EV_N.state(1,1), ...
    %                     training_data(i).EV_N.state(2,1), ...
    %                     training_data(i).EV_N.state(3,1), EV.width, EV.length, plt_ops);
    % hold on
    % plt_ops.color = 'black';
    % [p_TV, c_TV] = plotCar(training_data(i).TV_N.state(1,1), ...
    %                     training_data(i).TV_N.state(2,1), ...
    %                     training_data(i).TV_N.state(3,1), TV.width, TV.length, plt_ops);
    % hold on

    % === Uncomment this part to plot the vehicle states along the horizon
    t_EV = plot(training_data(i).EV_N.state(1,:), training_data(i).EV_N.state(2,:), 'rs');
    hold on

    t_TV = plot(training_data(i).TV_N.state(1,:), training_data(i).TV_N.state(2,:), 'bs');
    hold on
    
    axis equal
    axis(map_dim);
    pause(0.1)
%     input('Any key')
    
    % delete(p_EV)
    delete(t_EV)
    % delete(p_TV)
    delete(t_TV)
    delete(p_hyp)
    delete(p_hyp_hat)
end