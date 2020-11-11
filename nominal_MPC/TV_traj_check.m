close('all');
clear('all');
clc

%% Load .mat file
for exp_num = 1:486
    exp_file = strcat('../data/exp_num_', num2str(exp_num), '.mat');
    load(exp_file)
    
    close all
    
    figure
    
    subplot(2,1,1)
    title(num2str(exp_num))
    plot(TV.x, TV.y, 'linewidth', 2)
    subplot(2,1,2)
    plot(TV.v, 'linewidth', 2)
    
    pause
end