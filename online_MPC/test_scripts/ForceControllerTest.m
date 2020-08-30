clear('all');
clc

addpath('../dynamics')

exp_num = 30;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

N = 20;

u_lim = [0.35, 1];
u_rate_lim = [0.3, 3];
y_lim = 3.5;

model = {};

name = 'FP_HPPSolver';

%% Definition
dt = EV.dt;

model.xinitidx = [1:4, 7:8];
% model.xinit = zeros(6, 1);

model.N = N + 1;

% ref_z has 4 elements, hyperplane has 3 elements
model.npar = 4 + 3;

model.eq = @dynamics;

for k = 1:model.N

	if k < model.N-1
		% z: [x, y, heading, v, delta, a, delta_prev, a_prev]
		model.nvar(k) = 8;
		
		% ineq: hpp and rate constraints
		model.nh(k) = 3;

		model.ineq{k} = @ineq;
		model.hu{k} = [inf, u_rate_lim*dt];
		model.hl{k} = [0, -u_rate_lim*dt];

		model.ub{k} = [inf, y_lim, inf, inf, ...
							u_lim, ...
							inf, inf];
		model.lb{k} = -model.ub{k};

		% eq: dynamics for (x, y, heading, v, delta_prev, a_prev)
		model.neq(k) = 6;

		model.E{k} = [1 0 0 0 0 0 0 0;
						   0 1 0 0 0 0 0 0;
						   0 0 1 0 0 0 0 0;
						   0 0 0 1 0 0 0 0;
						   0 0 0 0 0 0 1 0;
						   0 0 0 0 0 0 0 1];

		model.objective{k} = @objective;
	elseif k == model.N-1
		% z: [x, y, heading, v, delta, a, delta_prev, a_prev]
		model.nvar(k) = 8;
		
		% ineq: rate constraints and hpp
		model.nh(k) = 3;

		model.ineq{k} = @ineq;
		model.hu{k} = [inf, u_rate_lim*dt];
		model.hl{k} = [0, -u_rate_lim*dt];

		model.ub{k} = [inf, y_lim, inf, inf, ...
							u_lim, ...
							inf, inf];
		model.lb{k} = -model.ub{k};

		% eq: dynamics for (x, y, heading, v, delta_prev, a_prev)
		model.neq(k) = 6;

		model.E{k} = [eye(4); zeros(2, 4)];

		model.objective{k} = @objective;
	else
		% z: [x, y, heading, v]
		model.nvar(k) = 4;

		% ineq: hpp
		model.nh(k) = 1;

		model.ineq{k} = @ineqN;
		model.hu{k} = inf;
		model.hl{k} = 0;

		model.ub{k} = [inf, y_lim, inf, inf];

		model.lb{k} = -model.ub{k};

		model.objective{k} = @objectiveN;
	end

end

%% Generate Solver
codeoptions = getOptions(name);
codeoptions.maxit = 300;    % Maximum number of iterations
codeoptions.printlevel = 1; % Use printlevel = 1 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

codeoptions.nlp.TolStat = 1e-2;
codeoptions.nlp.TolEq = 1e-3;
codeoptions.nlp.TolIneq = 1e-3;

codeoptions.nlp.ad_tool = 'casadi-351';
codeoptions.BuildSimulinkBlock = 0;

FORCES_NLP(model, codeoptions);


%% Function definitions

%% dynamics: dynamic model
function zp1 = dynamics(z)

	% Dynamics
	L_r = 1.4714;
	L_f = 1.4714;
	dt = 0.1;
	M = 10; % RK4 steps

	bike_dyn = bike_dynamics_rk4(L_r, L_f, dt, M);

	x = [z(1); z(2); z(3); z(4)];
	u = [z(5); z(6)];

	xp1 = bike_dyn.f_dt(x, u);

	zp1 = [xp1; u];
end

%% ineq: ineq constraints at normal stages
function h = ineq(z, p)
	x = z(1:4);
	u = z(5:6);
	u_prev = z(7:8);

	hyp = p(5:7);

	h = [hyp(1)*x(1) + hyp(2)*x(2) - hyp(3); ...
		 u(1) - u_prev(1); ...
		 u(2) - u_prev(2)];

end

%% ineqN: ineq constraints at terninal stage
function h = ineqN(z, p)
	x = z(1:4);

	hyp = p(5:7);

	h = hyp(1)*x(1) + hyp(2)*x(2) - hyp(3);
end

%% objective: stage objective
function obj = objective(z, p)
	z_coeff = [0.05, 0.1, 0.1, 0.5];
	u_coeff = [0.01, 0.01];

	x = z(1:4);
	u = z(5:6);

	z_ref = p(1:4);

	obj = u_coeff(1)*u(1)^2 + u_coeff(2)*u(2)^2;

	for i = 1:4
		obj = obj + z_coeff(i) * (x(i) - z_ref(i))^2;
	end

end

%% objectiveN: ternimal objective
function obj = objectiveN(z, p)
	z_coeff = [0.05, 0.1, 0.1, 0.5];

	x = z(1:4);

	z_ref = p(1:4);

	obj = 0;

	for i = 1:4
		obj = obj + z_coeff(i) * (x(i) - z_ref(i))^2;
	end
end