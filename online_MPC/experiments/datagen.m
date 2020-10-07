clear all
close all
clc

%% Data generation
pathsetup();

all_nums = 486;
data_gen = true;

all_col = zeros(1, all_nums);

unknown_error = zeros(1, all_nums);

all_times = zeros(1, all_nums);

parfor exp_num = 1:all_nums
	fprintf('Solving exp_num = %d\n', exp_num);
    try
        % [col, T_final] = FSM_HOBCA_strat_fp(exp_num, data_gen);
        [col, T_final] = FSM_HOBCA_strat_fp_inflated(exp_num, data_gen);
		all_col(exp_num) = col;
		all_times(exp_num) = T_final;
    catch
        fprintf('Unknown Error for exp_num = %d\n', exp_num)
        unknown_error(exp_num) = true;
    end
end

filename = sprintf('../datagen/0STATS_FSM_hobca_strat_inflated_%s.mat', datestr(now,'yyyy-mm-dd_HH-MM'));
save(filename, 'all_col', 'unknown_error', 'all_times')

fprintf('Stats saved as: %s, datagen of FSM_HOBCA_strat completed.\n', filename)

%% Analysis
strat_old = load('../datagen/0STATS_hobca_strat_2020-09-22_02-19.mat');
strat_FSM = load('../datagen/0STATS_FSM_hobca_strat_inflated1_lock20_ulim_2.5_8_2020-10-04_22-46.mat');
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

disp('FSM unfixed collisions:')
unfixed_collisions = find(strat_old.all_col + strat_FSM.all_col == 2);
disp(unfixed_collisions)

disp('FSM Produces new collisions:')
new_collisions = find(strat_old.all_col - strat_FSM.all_col == -1);
disp(new_collisions)

disp('FSM Produces unknown errors')
disp(find(strat_FSM.unknown_error == 1))

figure
histogram(naive.all_times(naive_col_free), 30, 'DisplayName', 'Naive HOBCA')
hold on
% histogram(strat_old.all_times(strat_old_col_free), 30, 'DisplayName', 'Strat Old')
% hold on
histogram(strat_FSM.all_times(strat_FSM_col_free), 30, 'DisplayName', 'Data-Driven FSM')
legend
title('Task Time Distribution')
xlabel('Time Steps')
ylabel('Number of Cases')


%% Generate Movies with Collision
files = dir('../datagen/');

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
plt_params.mv_path = '../../movies/datagen1002/';

for i = 1:length(col_filenames)
    name = col_filenames{i};
    fname = sprintf('../datagen/%s.mat', name{:});
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
plt_params.mv_path = '../../movies/datagen1002/';

exp_nums = [fixed_collisions, new_collisions];
for exp_num = exp_nums
    search_str = sprintf('../datagen/1002/*Exp%d_*.mat', exp_num);
    file = dir(search_str);
    if length(file) == 1
        fname = sprintf('../datagen/1002/%s', file.name);
        file_split = split(file.name, '.');
        plt_params.mv_name = file_split{1};
        
        F = plotExp(fname, plt_params);
    else
        fprintf('Exp %d file was not found. \n', exp_num)
    end
end