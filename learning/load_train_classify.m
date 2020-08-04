%% Load data and training
clear('all');
close('all');
clc

%% Load dataset file
% If reconstructing the features and labels is
% necessary, load the file named 'hpp_data_*.mat'

% If you want to directly load the previously 
% constructed feature-label, load the file named 
% 'trn_val*.mat'
[file, path] = uigetfile('../hyperplane_dataset/*.mat', 'Select Raw Dataset');
load([path, file])

%% Construct features and labels, if needed
% Otherwise skip these sections
% ======================================

% Select valid experiments
valid_exps = [];
for exp_num = 1:length(training_set)
	training_data = training_set{exp_num};

	if isempty(training_data)
		continue
	else
		valid_exps = [valid_exps, exp_num];
	end
end

% 80% training set, 20% validation/test set
valid_num = length(valid_exps);
trn_exps = randsample(valid_exps, floor(0.8*valid_num));
val_exps = setdiff(valid_exps, trn_exps);

% Training set
trn_feature = [];
trn_label   = [];

for i = 1:length(trn_exps)
	exp_num = trn_exps(i);
	fprintf('Traning set: %d / %d\n', i, length(trn_exps))

	training_data = training_set{exp_num};

	for t = 1:length(training_data)
		feature = training_data(t).X;
		label   = training_data(t).Y;

		% Append into the 3rd dim
		trn_feature = cat(3, trn_feature, feature);
		trn_label   = [trn_label, label];
	end
end

% Val set
val_feature = [];
val_label   = [];

for i = 1:length(val_exps)
	exp_num = val_exps(i);
	fprintf('Validation set: %d / %d\n', i, length(val_exps))

	training_data = training_set{exp_num};

	for t = 1:length(training_data)
		feature = training_data(t).X;
		label   = training_data(t).Y;

		% Append into the 3rd dim
		val_feature = cat(3, val_feature, feature);
		val_label   = [val_label, label];
	end
end

% Reshape
trn_size = size(trn_feature, 3);
trn_feature_flat = reshape(trn_feature, [], trn_size);
trn_label_flat   = reshape(trn_label, [], trn_size);

val_size = size(val_feature, 3);
val_feature_flat = reshape(val_feature, [], val_size);
val_label_flat   = reshape(val_label, [], val_size);

% Shuffle training set
col_perm = randperm(trn_size);
trn_feature_flat = trn_feature_flat(:, col_perm);
trn_label_flat   = trn_label_flat(:, col_perm);

% Combined set
clas_feature = [trn_feature_flat, val_feature_flat];
clas_label = [trn_label_flat, val_label_flat];

% Save
uisave({'trn_feature_flat', 'trn_label_flat', ...
		'val_feature_flat', 'val_label_flat', ...
		'clas_feature', 'clas_label', ...
		'trn_size', 'val_size', ...
		'trn_exps', 'val_exps'}, ...
		['../hyperplane_dataset/strat_trn_val_dataset_', ...
		datestr(now, 'yyyy-mm-dd_HH-MM'), '.mat'])