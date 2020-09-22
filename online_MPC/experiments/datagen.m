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
strat = load('../datagen/0STATS_hobca_strat_2020-09-21_17-25.mat');
naive = load('../datagen/0STATS_hobca_naive_2020-09-21_04-43.mat');

fprintf('Emergency Break: Strat = %d, Naive = %d \n', sum(strat.all_eb), sum(naive.all_eb));
fprintf('Actual Collision: Strat = %d, Naive = %d \n', sum(strat.all_col), sum(naive.all_col));
fprintf('Mean Task Time: Strat = %d, Naive = %d \n', mean(strat.all_times), mean(naive.all_times));
