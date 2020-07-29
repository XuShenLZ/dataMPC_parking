%% Load data and training
clear('all');
close('all');
clc

%% Load dataset file
% If reconstructing the features and labels is
% necessary, load the file named 'hpp_data_*'
% If you want to directly load the previously 
% constructed feature-label, load the file named 
% 'feature_label_*'
[file, path] = uigetfile('../hyperplane_dataset/*.mat', 'Select Raw Dataset');
load([path, file])

%% Construct features and labels, if needed
% Otherwise skip these sections
% ======================================
mid_point = true;
N = 20;

all_feature = [];
all_label   = [];

for exp_num = 1:length(training_set)
	fprintf('Exp_num: %d / %d\n', exp_num, length(training_set))

	training_data = training_set{exp_num};

	if isempty(training_data)
		continue
	end

	for t = 1:length(training_data)-N
		[feature, label, ~] = gen_feature_label(training_data, t, N, mid_point);

		% Append into the 3rd dim
		all_feature = cat(3, all_feature, feature);
		all_label   = cat(3, all_label, label);
	end
end

% Reshape
batch_size = size(all_feature, 3);

feature_flat = reshape(all_feature, [], batch_size);
label_flat   = reshape(all_label, [], batch_size);

% Shuffle
col_perm = randperm(batch_size);
feature_flat = feature_flat(:, col_perm);
label_flat   = label_flat(:, col_perm);

% Normalize
norm_feature_flat = feature_flat ./ vecnorm(feature_flat, 2, 1);
norm_label_flat   = label_flat ./ vecnorm(label_flat, 2, 1);

% Data for regression toolbox
reg_data = [feature_flat; label_flat]';

% Save
uisave({'feature_flat', 'label_flat', ...
		'norm_feature_flat', 'norm_label_flat', ...
		'reg_data', 'mid_point'}, ...
		['../hyperplane_dataset/reg_dataset_', ...
		datestr(now, 'yyyy-mm-dd_HH-MM')])

% ======================================

%% Train
model_name = 'midPoint';

% Neural Network and Save
% ===================
% hidden_size = 40;
% [net, tr] = nn(x, t, hidden_size, model_name);
% ===================

% Gaussian Process and Save
% ===================
feature_dim = size(feature_flat, 1);
label_dim   = size(label_flat, 1);

GPs = cell(1, label_dim);

parfor i = 1:label_dim
	fprintf('Training GP: #%d / %d\n', i, label_dim)

	[GPs{i}.trainedModel, GPs{i}.validation] = gp(reg_data, feature_dim, label_dim, i);
end

uisave('GPs', sprintf('models/GP_%s_%s.mat', ...
						model_name, ...
						datestr(now,'yyyy-mm-dd_HH-MM')) )