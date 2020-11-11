%% Predict the learned strategy using model
% and plot the posterior prob with the optimal one
clear('all');
close('all');
clc

addpath('../online_MPC')

%% Load Strategy Dataset, Prediction Model, and validation exps
model_type = "nn"; % knn, gSVM, bagTree, nn
[file, path] = uigetfile(sprintf('../models/%s*.mat', model_type), 'Select Prediction Model');
load([path, file])

[file, path] = uigetfile('../hyperplane_dataset/strategy*.mat', 'Select Raw Strategy Dataset');
load([path, file])
[file, path] = uigetfile('../hyperplane_dataset/strat_trn_val*.mat', 'Select Train-Val Dataset');
load([path, file], 'val_exps')
fprintf('The exp_nums in validation set are: [%s]\n', num2str(val_exps, '%d,'))

%% Load TV data
exp_num = 44;
load(['../../data/exp_num_', num2str(exp_num), '.mat'])

training_data = training_set{exp_num};
if isempty(training_data)
    error('This scenario is invalid. Please specify a different exp_num.');
end
T = length(training_data);
N = 20;

V = 0.01 * eye(3);
W = 0.5 * eye(3);
Pm = 0.2 * eye(3);
score_hat = ones(3, 1) / 3;

%% Predict the hyperplane using trained function
for t = 1:T-N
	feature = training_data(t).X;
     % Flatten
    feature_flat = reshape(feature, [], 1);

    predictor = feature_flat';

    switch model_type
        case "gSVM"
            [Y, ~, ~, score] = predict(trainedModel, predictor);
        case "nn"
            score = net(feature_flat);
            [~, max_idx] = max(score);
            if max_idx == 1
                Y = {"L"};
            elseif max_idx == 2
                Y = {"R"};
            else
                Y = {"Y"};
            end
        otherwise
            [Y, score] = predict(trainedModel, predictor);
    end

	predicted_data(t).Y = Y{:};
	predicted_data(t).score = score / sum(score);

    [score_hat, Pm] = score_KF(score_hat, score, V, W, Pm);
    predicted_data(t).score_hat = score_hat;
end

%% Plot
fig = figure();
ax1 = subplot(2,1,1);
plot(TV.x, TV.y, 'k--');
EV_line = animatedline(ax1, 'color', 'r', 'linestyle', 'none', 'marker', 's');
TV_line = animatedline(ax1, 'color', 'b', 'linestyle', 'none', 'marker', 's');
map_dim = [-30 30 -10 10];
hold on

ax2 = subplot(2,1,2);
L_line = animatedline(ax2, 'color', '#0072BD', 'linewidth', 2);
R_line = animatedline(ax2, 'color', '#D95319', 'linewidth', 2);
Y_line = animatedline(ax2, 'color', '#77AC30', 'linewidth', 2);

L_hat_line = animatedline(ax2, 'color', 'b', 'LineStyle', '--', 'linewidth', 2);
R_hat_line = animatedline(ax2, 'color', 'r', 'LineStyle', '--', 'linewidth', 2);
Y_hat_line = animatedline(ax2, 'color', 'g', 'LineStyle', '--', 'linewidth', 2);

legend('Left', 'Right', 'Yield', 'Left_hat', 'Right_hat', 'Yield_hat')
prob_dim = [1 T-N -0.2 1.2];
axis(prob_dim)

F(T-N) = struct('cdata',[],'colormap',[]);
for i = 1:T-N
    if EV.traj(1, i) > 30
        break
    end

    Y = training_data(i).Y;

    Y_hat = predicted_data(i).Y;
    score = predicted_data(i).score;
    score_hat = predicted_data(i).score_hat;

    addpoints(L_line, i, score(1));
    addpoints(R_line, i, score(2));
    addpoints(Y_line, i, score(3));
    addpoints(L_hat_line, i, score_hat(1));
    addpoints(R_hat_line, i, score_hat(2));
    addpoints(Y_hat_line, i, score_hat(3));

    if i>1
        clearpoints(EV_line);
        clearpoints(TV_line);
    end
    addpoints(EV_line, EV.traj(1, i:i+N), EV.traj(2, i:i+N));
    addpoints(TV_line, TV.x(i:i+N), TV.y(i:i+N));
    
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
    axes(ax1);
    % t_EV = plot(EV.traj(1, i:i+N), EV.traj(2, i:i+N), 'rs');
    % hold on

    % t_TV = plot(TV.x(i:i+N), TV.y(i:i+N), 'bs');
    % hold on
    
    t_Y = text(-25, 2, sprintf('Strategy: %s', Y), 'color', 'k');
    t_Y_hat = text(-25, -2, sprintf('Strategy: %s', Y_hat), 'color', 'm');
    axis equal
    axis(map_dim);

    axes(ax2)
    drawnow
    % pause(0.05)
    F(i) = getframe(fig);
%     input('Any key')
    
    % delete(p_EV)
    % delete(t_EV)
    % delete(p_TV)
    % delete(t_TV)

    delete(t_Y)
    delete(t_Y_hat)
end

%% Save Movie
if ~isfolder('../../movies/')
    mkdir('../../movies')
end

[file,path] = uiputfile(sprintf('../../movies/%s_Exp%d_%s.mp4', ...
                    model_type, exp_num, datestr(now,'yyyy-mm-dd_HH-MM')));

v = VideoWriter([path, file], 'MPEG-4');
v.FrameRate = 10;
open(v);
writeVideo(v,F);
close(v);