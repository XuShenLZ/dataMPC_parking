clear all
close all
clc

pathsetup();

exp_num = 44;
data_gen = false;

%% Use the FSM strat HOBCA
[col, T_final] = FSM_HOBCA_strat_fp(exp_num, data_gen);

%% Use the FSM naive HOBCA
% [col, T_final] = FSM_HOBCA_naive_fp(exp_num, data_gen);