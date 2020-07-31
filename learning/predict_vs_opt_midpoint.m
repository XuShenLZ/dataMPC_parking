%% Predict the learned hyperplane using network
% and plot it with the optimal one
clear('all');
close('all');
clc

%% Load Hyperplane Dataset, Prediction Model, and validation exps
[file, path] = uigetfile('../hyperplane_dataset/hpp*.mat', 'Select Raw Hyperplane Dataset');
load([path, file])
[file, path] = uigetfile('models/*.mat', 'Select Prediction Model');
load([path, file])
[file, path] = uigetfile('../hyperplane_dataset/trn_val*.mat', 'Select Train-Val Dataset');
load([path, file], 'val_exps')
fprintf('The exp_nums in validation set are: [%s]\n', num2str(val_exps, '%d,'))

%% Load TV data
exp_num = 2;
load(['../data/exp_num_', num2str(exp_num), '.mat'])

training_data = training_set{exp_num};
if isempty(training_data)
    error('This scenario is invalid. Please specify a different exp_num.');
end
T = length(training_data);
N = 20;

%% Predict the hyperplane using trained function
mid_point = true;
model_type = 'gp';
for t = 1:T-N
	[feature, ~, ~] = gen_feature_label(training_data, t, N, mid_point);
     % Flatten
    feature_flat = reshape(feature, [], 1);

    switch model_type
        case 'nn'
            % Normalize
            % feature_flat = feature_flat ./ vecnorm(feature_flat, 2, 1);

            % Predict
            Y = net(feature_flat);
        case 'gp'
            predictor = feature_flat';
            label_dim = 21;

            Y = zeros(label_dim, 1);
            for i = 1:label_dim
                Y(i) = predict(GPs{i}.trainedModel, predictor);
            end
        otherwise
            error('Model not defined');
    end
    % Return to shape 1 x timesteps
    angles = reshape(Y, [], N+1);

	% Obtain the [w, b] form of hyperplane
    global_hpp = zeros(3, N+1);
	for k = 0:N
        EV_xyt = training_data(t+k).EV_N.state(1:3, 1);
        TV_xyt = training_data(t+k).TV_N.state(1:3, 1);
        m = calc_midpoint(EV_xyt, TV_xyt, EV, TV);

        slope = tan(angles(k+1)); % Slope
        % y - y0 = s(x - x0)
        global_hpp(1, k+1) = slope;
        global_hpp(2, k+1) = -1;
		global_hpp(3, k+1) = -slope*m(1) + m(2);
	end

	predicted_data(t).hyperplane.w = global_hpp(1:2, 1);
	predicted_data(t).hyperplane.b = global_hpp(3, 1);
    predicted_data(t).hyperplane.s = angles(1);
end

%% Plot
fig = figure();
map_dim = [-30 30 -10 10];
plot(TV.x, TV.y, 'k--');
hold on

F(T-N) = struct('cdata',[],'colormap',[]);
for i = 1:T-N
    w = training_data(i).hyperplane.w;
    b = training_data(i).hyperplane.b;
    m = training_data(i).hyperplane.m;
    s = training_data(i).hyperplane.s;

    hyp_y = [-10, 10];
    hyp_x = -(w(2)*hyp_y+b)/w(1);

    p_hyp = plot(hyp_x, hyp_y, 'k');
    hold on

    w_hat = predicted_data(i).hyperplane.w;
    b_hat = predicted_data(i).hyperplane.b;
    s_hat = predicted_data(i).hyperplane.s;

    hyp_y_hat = [-10, 10];
    hyp_x_hat = -(w_hat(2)*hyp_y_hat+b_hat)/w_hat(1);

    p_hyp_hat = plot(hyp_x_hat, hyp_y_hat, 'm--');
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
    
    m_hyp = plot(m(1), m(2), 'go');
    hold on
    t_s = text(m(1)+2, m(2)+2, sprintf('slope: %g', s), 'color', 'k');
    t_s_hat = text(m(1)+2, m(2)-2, sprintf('slope: %g', s_hat), 'color', 'm');
    axis equal
    axis(map_dim);
    pause(0.2)
    F(i) = getframe;
%     input('Any key')
    
    % delete(p_EV)
    delete(t_EV)
    % delete(p_TV)
    delete(t_TV)
    delete(p_hyp)
    delete(p_hyp_hat)
    delete(m_hyp)
    delete(t_s)
    delete(t_s_hat)
end

%% Save Movie
if exist('./movies/') ~= 7
    mkdir('movies')
end

[file,path] = uiputfile(sprintf('movies/%s_Exp%d_%s.mp4', ...
                    model_type, exp_num, datestr(now,'yyyy-mm-dd_HH-MM')));

v = VideoWriter([path, file], 'MPEG-4');
v.FrameRate = 10;
open(v);
writeVideo(v,F);
close(v);