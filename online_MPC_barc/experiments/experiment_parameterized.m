clear all
close all
clc

pathsetup();

exp_num = 4;
datagen = false;

%% Use the FSM strat HOBCA
% [col, T_final] = FSM_HOBCA_strat_fp_inflated(exp_num, datagen);

%% Use the FSM naive HOBCA
[col, T_final] = FSM_HOBCA_naive_fp_parameterized(exp_num, datagen);

%% Test tracking NMPC
% tracking_fp(exp_num, datagen);