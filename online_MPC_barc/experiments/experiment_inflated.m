clear all
close all
clc

pathsetup();

exp_num = 4;
datagen = false;

%% Use the FSM strat HOBCA
% [col, T_final] = FSM_HOBCA_strat_fp_inflated(exp_num, datagen);
[col, T_final] = FSM_HOBCA_strat_fp_inflated_static_TV(exp_num, datagen);

%% Use the FSM naive HOBCA
% [col, T_final] = FSM_HOBCA_naive_fp(exp_num, datagen);
% [col, T_final] = FSM_HOBCA_naive_fp_static_TV(exp_num, datagen);

%% Test tracking NMPC
% tracking_fp(exp_num, datagen);