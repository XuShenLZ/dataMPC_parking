%% Load data and training
clear('all');
close('all');
clc

rng(0);

%% Load dataset file
% Select 'strat_trn_val_dataset_*.mat'
[file, path] = uigetfile('../hyperplane_dataset/*.mat', 'Select 1st Dataset');
D1 = load([path, file]);

[file, path] = uigetfile('../hyperplane_dataset/*.mat', 'Select 2nd Dataset');
D2 = load([path, file]);

%%
trn_exps = D1.trn_exps;
val_exps = D1.val_exps;

% Combine
trn_size = D1.trn_size + D2.trn_size;
val_size = D1.val_size + D2.val_size;

trn_feature_flat = [D1.trn_feature_flat, D2.trn_feature_flat];
trn_label_flat = [D1.trn_label_flat, D2.trn_label_flat];
val_feature_flat = [D1.val_feature_flat, D2.val_feature_flat];
val_label_flat = [D1.val_label_flat, D2.val_label_flat];
trn_label_onehot = [D1.trn_label_onehot, D2.trn_label_onehot];
val_label_onehot = [D1.val_label_onehot, D2.val_label_onehot];

% Randomize
col_perm = randperm(trn_size);

trn_feature_flat = trn_feature_flat(:, col_perm);
trn_label_flat = trn_label_flat(:, col_perm);
trn_label_onehot = trn_label_onehot(:, col_perm);

% Make the ensemble set
clas_feature = [trn_feature_flat, val_feature_flat];
clas_label = [trn_label_flat, val_label_flat];
clas_label_onehot = [trn_label_onehot, val_label_onehot];

% Save
uisave({'trn_feature_flat', 'trn_label_flat', ...
		'val_feature_flat', 'val_label_flat', ...
		'trn_label_onehot', 'val_label_onehot', ...
		'clas_feature', 'clas_label', 'clas_label_onehot', ...
		'trn_size', 'val_size', ...
		'trn_exps', 'val_exps'}, ...
		['../hyperplane_dataset/strat_trn_val_dataset_', ...
		datestr(now, 'yyyy-mm-dd_HH-MM'), '_aug.mat'])
