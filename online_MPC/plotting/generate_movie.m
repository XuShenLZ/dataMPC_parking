close all
clear all

% Plot and save movie

name = 'FSM_HOBCA_strat_fp_Exp4_Col0_2021-05-25_16-19';
fname = sprintf('../data/%s.mat', name);

plt_params.visible = 'on'; % or 'off' to shut down real time display
plt_params.plt_hyp = true;
plt_params.plt_ref = true;
plt_params.plt_sol = false;
plt_params.plt_preds = true;
plt_params.plt_tv_preds = true;
plt_params.plt_col_buf = false;
plt_params.mv_save = true;
plt_params.mv_name = name;

F = plotExp(fname, plt_params);