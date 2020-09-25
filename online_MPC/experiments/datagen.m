clear all
close all
clc

%% Data generation
% datagen_HOBCA_strat_fp;

% datagen_HOBCA_naive_fp;

% backup controller will activate when infeasible
% and emergency break will activate when there is actual collision
BACK_datagen_HOBCA_strat_fp;

BACK_datagen_HOBCA_naive_fp;

%% Analysis
strat = load('../datagen/0STATS_hobca_strat_2020-09-22_02-19.mat');
naive = load('../datagen/0STATS_hobca_naive_2020-09-22_03-11.mat');

fprintf('Emergency Break: Strat = %d, Naive = %d \n', sum(strat.all_eb), sum(naive.all_eb));
fprintf('Actual Collision: Strat = %d, Naive = %d \n', sum(strat.all_col), sum(naive.all_col));
fprintf('Mean Task Time: Strat = %d, Naive = %d \n', mean(strat.all_times), mean(naive.all_times));

figure
histogram(naive.all_times, 30, 'DisplayName', 'Naive')
hold on
histogram(strat.all_times, 30, 'DisplayName', 'Strat')
legend
title('Task Time Distribution')

%% Generate Movies with Collision
files = dir('../datagen/0922/');

col_filenames = {};
for i = 3:length(files)
   name = regexp(files(i).name, '(.+strat.+Col1.+)\.mat', 'tokens');
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
plt_params.mv_path = '../../movies/datagen0922/';

for i = 1:length(col_filenames)
    name = col_filenames{i};
    fname = sprintf('../datagen/0922/%s.mat', name{:});
    plt_params.mv_name = name{:};
    
    F = plotExp(fname, plt_params);
end
