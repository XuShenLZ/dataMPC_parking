clear('all');
close('all');
clc

%% ====== Generate data 

all_nums = 486;
N = 20;

%% ====== Load data set that is to be appended by yield data
load('../hyperplane_dataset_07-28-2020/strategy_data_2020-08-03_17-11')

exp_num = 487;

% numR = 0; 
% numL = 0; 
% numY = 0; 
% 
% for i = 1:486
%     for j = 1: size(training_set{1,i},2)
% idxR = strfind([training_set{1,i}.Y], 'R');
% idxL = strfind([training_set{1,i}.Y], 'L');
% idxY = strfind([training_set{1,i}.Y], 'Y');
% numR = numR + length(find(cell2mat(idxR)~=0)); % 11498807
% numL = numL + length(find(cell2mat(idxL)~=0)); % 14989622
% numY = numY + length(find(cell2mat(idxY)~=0)); %  1062515
%     end
% end


%% generate fake data 
label = "Y"; % Yield

epsth = pi/6;
ref_va_TV = -1;
ref_vb_TV = 1;

ref_va_EV = 0;
ref_vb_EV = 2;

posxa = 8;
posxb = 15;
posyc = -4.8;
posyd = 4.8;

a = -epsth-0.5*pi;
b = epsth-0.5*pi;
c = -epsth+0.5*pi;
d = epsth+0.5*pi;

e = ref_va_TV;
f = ref_vb_TV;
g = ref_va_EV;
h = ref_vb_EV;


for exp_num = 487:700
    
for i = 1:100
    
x_gen = posxa + (posxb-posxa) * rand(N+1,1);
y_gen = posyc + (posyd-posyc) * rand(N+1,1);
th_gen_TV = a + (b-a) * rand(N+1,1);
th_gen_EV = 0; 
v_gen_TV = e + (f-e) * rand(N+1,1);
v_gen_EV = g + (h-g) * rand(N+1,1);

vx_gen_TV = v_gen_TV.*cos(th_gen_TV); 
vy_gen_TV = v_gen_TV.*sin(th_gen_TV);

vx_gen_EV = v_gen_EV.*cos(th_gen_EV); 
vy_gen_EV = v_gen_EV.*sin(th_gen_EV);

vx_gen = vx_gen_TV - vx_gen_EV;
vy_gen = vy_gen_TV - vy_gen_EV;

th_gen = th_gen_TV - th_gen_EV;

rel_state = [x_gen'; y_gen'; th_gen'; vx_gen'; vy_gen'];

training_data(i).X = rel_state;
training_data(i).Y = label; 

end 

for i = 101:200 
    
x_gen = posxa + (posxb-posxa) * rand(N+1,1);
y_gen = posyc + (posyd-posyc) * rand(N+1,1);
th_gen = c + (d-c) * rand(N+1,1);
vx_gen = e + (f-e) * rand(N+1,1);
vy_gen = g + (h-g) * rand(N+1,1);

rel_state = [x_gen'; y_gen'; th_gen'; vx_gen'; vy_gen'];

training_data(i).X = rel_state;
training_data(i).Y = label; 

end


training_set{exp_num} = training_data;

exp_num = exp_num + 1; 

end



dataset_filename = ['../hyperplane_dataset_07-28-2020/strategy_data_2020-08-03_17-11_appended_yield.mat'];

save(dataset_filename, 'training_set')




