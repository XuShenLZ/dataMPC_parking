function generate_forces_pro_opt_solver_slack_hyp(params)
    global n_x n_u n_obs n_ineq m_ineq d_ineq N_ineq M_ineq G g Q R alpha dynamics
    
    n_x = params.n_x;
    n_u = params.n_u;
    n_obs = params.n_obs;
    n_ineq = params.n_ineq;
    d_ineq = params.d_ineq;
    G = params.G;
    g = params.g;
    m_ineq = size(G,1);
    Q = params.Q;
    R = params.R;
    alpha = params.alpha;
    dt = params.dt;
    dynamics = params.dynamics;
    d_min = params.d_min;
    u_l = params.u_l;
    u_u = params.u_u;
    du_l = params.du_l;
    du_u = params.du_u;
    
    N_ineq = sum(n_ineq);
    M_ineq = n_obs*m_ineq; 
    
    opt_model = {};
    
    opt_model.N = params.N + 1;
    
    nvar = zeros(opt_model.N,1); 
    neq = zeros(opt_model.N-1,1); 
    nh = zeros(opt_model.N,1); 
    npar = zeros(opt_model.N,1); 
    objective = cell(opt_model.N,1);
    eq = cell(opt_model.N-1,1);
    E = cell(opt_model.N-1,1);
    ineq = cell(opt_model.N,1);
    hu = cell(opt_model.N,1); 
    hl = cell(opt_model.N,1);
    ub = cell(opt_model.N,1);
    lb = cell(opt_model.N,1);
    
    for i = 1:opt_model.N-1
        objective{i} = @eval_opt_obj;
        
        % [x_k, lambda_k, mu_k, u_k, u_km1, eps_k]
        nvar(i) = n_x + N_ineq + M_ineq + n_u + n_u + n_obs;
        ub{i} = [inf*ones(n_x,1); inf*ones(N_ineq+M_ineq,1); u_u; u_u; inf*ones(n_obs,1)];
        lb{i} = [-inf*ones(n_x,1);  zeros(N_ineq+M_ineq,1); u_l; u_l; zeros(n_obs,1)];
        
        % [obca_d, obca_norm, du]
        nh(i) = n_obs + n_obs + n_u;
        ineq{i} = @eval_opt_ineq;
        hu{i} = [inf*ones(n_obs,1); ones(n_obs,1); dt*du_u];
        hl{i} = [d_min*ones(n_obs,1); -inf*ones(n_obs,1); dt*du_l];
        
        % [x_ref, obs_A, obs_b]
        npar(i) = n_x + N_ineq*d_ineq + N_ineq;
        
        if i == opt_model.N-1
            % [dynamics, obca]
            neq(i) = n_x + n_obs*d_ineq;
            E{i} = [eye(n_x), zeros(n_x, N_ineq + M_ineq + n_obs);
                zeros(n_obs*d_ineq, n_x + N_ineq + M_ineq + n_obs)];
            eq{i} = @eval_opt_eq_Nm1;
        else
            % [augmented dynamics, obca]
            neq(i) = n_x + n_u + n_obs*d_ineq;
            E{i} = [eye(n_x), zeros(n_x, N_ineq + M_ineq + n_u + n_u + n_obs);
                zeros(n_u,  n_x + N_ineq + M_ineq + n_u), eye(n_u), zeros(n_u, n_obs);
                zeros(n_obs*d_ineq, n_x + N_ineq + M_ineq + n_u + n_u + n_obs)];
            eq{i} = @eval_opt_eq;
        end
    end
    objective{opt_model.N} = @eval_opt_obj_N;
    
    % [x_k, lambda_k, mu_k, eps_k]
    nvar(opt_model.N) = n_x + N_ineq + M_ineq + n_obs;
    ub{opt_model.N} = [inf*ones(n_x,1); inf*ones(N_ineq + M_ineq,1); inf*ones(n_obs,1)];
    lb{opt_model.N} = [-inf*ones(n_x,1); zeros(N_ineq + M_ineq,1); zeros(n_obs,1)];
    
    % [obca_d, obca_norm]
    nh(opt_model.N) = n_obs + n_obs;
    ineq{opt_model.N} = @eval_opt_ineq_N;
    hu{opt_model.N} = [inf*ones(n_obs,1); ones(n_obs,1)];
    hl{opt_model.N} = [d_min*ones(n_obs,1); -inf*ones(n_obs,1)];
    
    % [x_ref, obs_A, obs_b]
    npar(opt_model.N) = n_x + N_ineq*d_ineq + N_ineq;
    
    opt_model.nvar = nvar;
    opt_model.neq = neq;
    opt_model.nh = nh;
    opt_model.npar = npar;

    opt_model.objective = objective;
    opt_model.eq = eq;
    opt_model.E = E;
    opt_model.ineq = ineq;
    opt_model.hu = hu;
    opt_model.hl = hl;
    opt_model.ub = ub;
    opt_model.lb = lb;
    
    opt_model.xinitidx = [1:n_x, n_x+N_ineq+M_ineq+n_u+1:n_x+N_ineq+M_ineq+n_u+n_u];
    
    opt_codeopts = getOptions(params.name);
%     opt_codeopts.maxit = 300;
    opt_codeopts.overwrite = 1;
    opt_codeopts.printlevel = 2;
    opt_codeopts.optlevel = 3;
    % opt_codeopts.init = 2;
    opt_codeopts.BuildSimulinkBlock = 0;

    opt_codeopts.nlp.ad_tool = 'casadi-351';
    opt_codeopts.nlp.linear_solver = 'symm_indefinite';

%     opt_codeopts.nlp.TolStat = 1e-3;
%     opt_codeopts.nlp.TolEq = 1e-3;
%     opt_codeopts.nlp.TolIneq = 1e-3;
    % opt_codeopts.nlp.hessian_approximation = 'gauss-newton';
    % opt_codeopts.nlp.BarrStrat = 'monotone';

%     opt_codeopts.linesearch.minstep = 1e-8;
%     opt_codeopts.linesearch.maxstep = 0.9;

%     opt_codeopts.nlp.reg_eta_dw = 1;
%     opt_codeopts.nlp.reg_beta_dw = 10;
%     opt_codeopts.nlp.reg_min_dw = 1;
%     opt_codeopts.nlp.reg_gamma_dw = 3;
% 
%     opt_codeopts.nlp.reg_eta_dc = 1;
%     opt_codeopts.nlp.reg_beta_dc = 10;
%     opt_codeopts.nlp.reg_min_dc = 1;
%     opt_codeopts.nlp.reg_gamma_dc = 3;
    
%     opt_codeopts.regularize.epsilon = 1E-12; % (for Hessian approx.)
%     opt_codeopts.regularize.delta = 4E-6; % (for Hessian approx.)
%     opt_codeopts.regularize.epsilon2 = 1E-14; % (for Normal eqs.)
%     opt_codeopts.regularize.delta2 = 1E-14; % (for Normal eqs.)

    FORCES_NLP(opt_model, opt_codeopts);
end

% Stage cost
function opt_obj = eval_opt_obj(z, p)
    global n_obs N_ineq M_ineq n_x n_u Q R alpha
    
    x = z(1:n_x);
    u = z(n_x+N_ineq+M_ineq+1:n_x+N_ineq+M_ineq+n_u);
    eps = z(n_x+N_ineq+M_ineq+n_u+n_u+1:n_x+N_ineq+M_ineq+n_u+n_u+n_obs);
    
    x_ref = p(1:n_x);
    opt_obj = bilin(Q, x-x_ref, x-x_ref) + bilin(R, u, u) - alpha*sum1(eps);
end

% Terminal cost
function opt_obj = eval_opt_obj_N(z, p)
    global n_obs N_ineq M_ineq n_x Q alpha
    
    x = z(1:n_x);
    x_ref = p(1:n_x);
    eps = z(n_x+N_ineq+M_ineq+1:n_x+N_ineq+M_ineq+n_obs);
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref) - alpha*sum1(eps);
end

% Equality constraints at each stage
function opt_eq = eval_opt_eq(z, p)
    global n_x n_u n_obs n_ineq m_ineq d_ineq N_ineq M_ineq G dynamics
    
    x = z(1:n_x);
    u = z(n_x+N_ineq+M_ineq+1:n_x+N_ineq+M_ineq+n_u);
    
    R_opt = [cos(z(3)), -sin(z(3)); sin(z(3)), cos(z(3))];

    j = 0;
    opt_eq = dynamics.f_dt_aug(x, u);
    for i = 1:n_obs
        A = reshape(p(n_x+j+1:n_x+j+n_ineq(i)*d_ineq), n_ineq(i), d_ineq);
        lambda = z(n_x+j+1:n_x+j+n_ineq(i));
        mu = z(n_x+N_ineq+(i-1)*m_ineq+1:n_x+N_ineq+i*m_ineq);

        opt_eq = vertcat(opt_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_opt)), lambda));
        
        j = j + n_ineq(i);
    end
end

% Equality constraints at last stage
function opt_eq = eval_opt_eq_Nm1(z, p)
    global n_x n_u n_obs n_ineq m_ineq d_ineq N_ineq M_ineq G dynamics
    
    x = z(1:n_x);
    u = z(n_x+N_ineq+M_ineq+1:n_x+N_ineq+M_ineq+n_u);
    
    R_opt = [cos(z(3)), -sin(z(3)); sin(z(3)), cos(z(3))];

    j = 0;
    opt_eq = dynamics.f_dt(x, u);
    for i = 1:n_obs
        A = reshape(p(n_x+j+1:n_x+j+n_ineq(i)*d_ineq), n_ineq(i), d_ineq);
        lambda = z(n_x+j+1:n_x+j+n_ineq(i));
        mu = z(n_x+N_ineq+(i-1)*m_ineq+1:n_x+N_ineq+i*m_ineq);
        
        opt_eq = vertcat(opt_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_opt)), lambda));
        
        j = j + n_ineq(i);
    end
end

% Inequality constraints at each stage
function opt_ineq = eval_opt_ineq(z, p)
    global n_x n_u n_obs n_ineq d_ineq m_ineq g

    u = z(n_x+N_ineq+M_ineq+1:n_x+N_ineq+M_ineq+n_u);
    u_p = z(n_x+N_ineq+M_ineq+n_u+1:n_x+N_ineq+M_ineq+n_u+n_u);
    
    t_opt = z(1:2);
    
    opt_ineq = [];
    j = 0;
    for i = 1:n_obs
        A = reshape(p(n_x+j+1:n_x+j+n_ineq(i)*d_ineq), n_ineq(i), d_ineq);
        b = p(n_x+N_ineq*d_ineq+j+1:n_x+N_ineq*d_ineq+j+n_ineq(i));    
        lambda = z(n_x+j+1:n_x+j+n_ineq(i));
        mu = z(n_x+N_ineq+(i-1)*m_ineq+1:n_x+N_ineq+i*m_ineq);
        
        opt_ineq = vertcat(opt_ineq, -dot(g,mu)+mtimes(transpose(mtimes(A,t_opt)-b),lambda));
        opt_ineq = vertcat(opt_ineq, dot(mtimes(transpose(A),lambda), mtimes(transpose(A),lambda))); 
        
        j = j + n_ineq(i);
    end
    
    opt_ineq = vertcat(opt_ineq, u-u_p);
end

% Inequality constraints at last stage
function opt_ineq = eval_opt_ineq_N(z, p)
    global n_x n_obs n_ineq d_ineq m_ineq g
    
    t_opt = z(1:2);
    
    opt_ineq = [];
    j = 0;
    for i = 1:n_obs
        A = reshape(p(n_x+j+1:n_x+j+n_ineq(i)*d_ineq), n_ineq(i), d_ineq);
        b = p(n_x+N_ineq*d_ineq+j+1:n_x+N_ineq*d_ineq+j+n_ineq(i));    
        lambda = z(n_x+j+1:n_x+j+n_ineq(i));
        mu = z(n_x+N_ineq+(i-1)*m_ineq+1:n_x+N_ineq+i*m_ineq);
        
        opt_ineq = vertcat(opt_ineq, -dot(g, mu)+mtimes(transpose(mtimes(A,t_opt)-b),lambda));
        opt_ineq = vertcat(opt_ineq, dot(mtimes(transpose(A), lambda), mtimes(transpose(A),lambda)));
        
        j = j + n_ineq(i);
    end
end