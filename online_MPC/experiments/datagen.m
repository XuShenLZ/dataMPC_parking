clear all
close all
clc

%% Data generation
FSM_datagen_HOBCA_strat_fp;

% BACK_datagen_HOBCA_naive_fp;

%% Analysis
strat_old = load('../datagen/0STATS_hobca_strat_2020-09-22_02-19.mat');
strat_FSM = load('../datagen/0STATS_FSM_hobca_strat_2020-09-29_21-15.mat');
naive = load('../datagen/0STATS_hobca_naive_2020-09-22_03-11.mat');

naive_col_free = find(naive.all_col == 0);
strat_old_col_free = find(strat_old.all_col == 0);
strat_FSM_col_free = find(strat_FSM.all_col == 0);

fprintf('Actual Collision: Old Strat = %d, FSM Strat = %d, Naive = %d \n\n', sum(strat_old.all_col), sum(strat_FSM.all_col), sum(naive.all_col));

fprintf('Mean Task Time: Old Strat = %d, FSM Strat = %d, Naive = %d \n\n', mean(strat_old.all_times(strat_old_col_free)), ...
                                                                        mean(strat_FSM.all_times(strat_FSM_col_free)), ...
                                                                        mean(naive.all_times(naive_col_free)));
                                                                    
fprintf('Median Task Time: Old Strat = %d, FSM Strat = %d, Naive = %d \n\n', median(strat_old.all_times(strat_old_col_free)), ...
                                                                        median(strat_FSM.all_times(strat_FSM_col_free)), ...
                                                                        median(naive.all_times(naive_col_free)));

disp('FSM fixes these collisions:')
fixed_collisions = find(strat_old.all_col - strat_FSM.all_col == 1);
disp(fixed_collisions)

disp('FSM Produces new collisions:')
new_collisions = find(strat_old.all_col - strat_FSM.all_col == -1);
disp(new_collisions)

disp('FSM Produces unknown errors')
disp(find(strat_FSM.unknown_error == 1))

figure
histogram(naive.all_times(naive_col_free), 30, 'DisplayName', 'Naive')
hold on
histogram(strat_old.all_times(strat_old_col_free), 30, 'DisplayName', 'Strat Old')
hold on
histogram(strat_FSM.all_times(strat_FSM_col_free), 30, 'DisplayName', 'Strat FSM')
legend
title('Task Time Distribution')

%% Generate Movies with Collision
files = dir('../datagen/0929/');

col_filenames = {};
for i = 3:length(files)
   name = regexp(files(i).name, '(FSM.+Col1.*)\.mat', 'tokens');
   if ~isempty(name)
       col_filenames{end+1} = name{1};
   end
end

fprintf('There are %d files found to have collision.\n', length(col_filenames));

plt_params.visible = 'off'; % or 'off' to shut down real time display
plt_params.plt_hyp = false;
plt_params.plt_ref = true;
plt_params.plt_sol = false;
plt_params.plt_preds = true;
plt_params.plt_tv_preds = true;
plt_params.plt_col_buf = false;
plt_params.mv_save = true;
plt_params.mv_path = '../../movies/datagen0929/';

for i = 1:length(col_filenames)
    name = col_filenames{i};
    fname = sprintf('../datagen/0929/%s.mat', name{:});
    plt_params.mv_name = name{:};
    
    F = plotExp(fname, plt_params);
end

%% Generate Movies with specific exp_num
plt_params.visible = 'off'; % or 'off' to shut down real time display
plt_params.plt_hyp = false;
plt_params.plt_ref = true;
plt_params.plt_sol = false;
plt_params.plt_preds = true;
plt_params.plt_tv_preds = true;
plt_params.plt_col_buf = false;
plt_params.mv_save = true;
plt_params.mv_path = '../../movies/datagen0929/';

exp_nums = [fixed_collisions, new_collisions];
for exp_num = exp_nums
    search_str = sprintf('../datagen/0929/*Exp%d_*.mat', exp_num);
    file = dir(search_str);
    if length(file) == 1
        fname = sprintf('../datagen/0929/%s', file.name);
        file_split = split(file.name, '.');
        plt_params.mv_name = file_split{1};
        
        F = plotExp(fname, plt_params);
    else
        fprintf('Exp %d file was not found. \n', exp_num)
    end
end