close all
clear all

% Plot and save movie

fname = '../data/FP_StratOBCA_Exp1_2020-09-14_17-48.mat';

plt_params.visible = 'on'; % or 'off' to shut down real time display
plt_params.plt_hyp = false;
plt_params.plt_ref = false;
plt_params.plt_sol = false;
plt_params.plt_preds = true;
plt_params.mv_save = true;
plt_params.mv_name = 'FP_StratOBCA';

F = plotExp(fname, plt_params);