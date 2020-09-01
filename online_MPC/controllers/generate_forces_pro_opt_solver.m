function generate_forces_pro_opt_solver(params)
    global n_x n_u n_obs n_ineq d_ineq G g Q R dynamics
    
    n_x = params.n_x;
    n_u = params.n_u;
    n_obs = params.n_obs;
    n_ineq = params.n_ineq;
    d_ineq = params.d_ineq;
    G = params.G;
    g = params.g;
    Q = params.Q;
    R = params.R;
    dt = params.dt;
    dynamics = params.dynamics;
    d_min = params.d_min;
    u_l = params.u_l;
    u_u = params.u_u;
    du_l = params.du_l;
    du_u = params.du_u;
    
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
    
    for i = 1:opt_model.N-1
        % [z_k, lambda_k, mu_k, u_k, u_km1]
        nvar(i) = n_x + n_obs*n_ineq + n_obs*n_ineq + n_u + n_u;
        % [lambda_k, mu_k, obca_d, obca_norm, hyp]
        nh(i) = n_obs*n_ineq + n_obs*n_ineq + n_obs + n_obs + 1 + n_u + n_u;
        % [z_ref, obs_A, obs_b, hyp_w, hyp_b]
        npar(i) = n_x + n_obs*n_ineq*d_ineq + n_obs*n_ineq + n_x + 1;
        
        if i == opt_model.N-1
            % [dynamics, obca]
            neq(i) = n_x + n_obs*d_ineq;
            E{i} = [eye(n_x), zeros(n_x, n_obs*n_ineq + n_obs*n_ineq);
                zeros(d_ineq, n_x + n_obs*n_ineq + n_obs*n_ineq)];
            eq{i} = @eval_opt_eq_Nm1;
        else
            % [augmented dynamics, obca]
            neq(i) = n_x + n_u + n_obs*d_ineq;
            E{i} = [eye(n_x), zeros(n_x, n_obs*n_ineq + n_obs*n_ineq + n_u + n_u);
                zeros(n_u,  n_x + n_obs*n_ineq + n_obs*n_ineq + n_u), eye(n_u);
                zeros(d_ineq, n_x + n_obs*n_ineq + n_obs*n_ineq + n_u + n_u)];
            eq{i} = @eval_opt_eq;
        end

        objective{i} = @eval_opt_obj;
        ineq{i} = @eval_opt_ineq;
        hu{i} = [inf*ones(n_obs*n_ineq+n_obs*n_ineq,1); inf*ones(n_obs,1); ones(n_obs,1); inf; u_u; dt*du_u];
        hl{i} = [zeros(n_obs*n_ineq+n_obs*n_ineq,1); d_min*ones(n_obs,1); -inf*ones(n_obs,1); 0; u_l; dt*du_l];
    end
    % [z_k, lambda_k, mu_k]
    nvar(opt_model.N) = n_x + n_obs*n_ineq + n_obs*n_ineq;
    % [lambda_k, mu_k, obca_d, obca_norm, hyp, u_k, du_k]
    nh(opt_model.N) = n_obs*n_ineq + n_obs*n_ineq + n_obs + n_obs + 1;
    % [z_ref, obs_A, obs_b, hyp_w, hyp_b]
    npar(opt_model.N) = n_x + n_obs*n_ineq*d_ineq + n_obs*n_ineq + n_x + 1;
    
    objective{opt_model.N} = @eval_opt_obj_N;
    ineq{opt_model.N} = @eval_opt_ineq_N;
    hu{opt_model.N} = [inf*ones(n_obs*n_ineq+n_obs*n_ineq,1); inf*ones(n_obs,1); ones(n_obs,1); inf];
    hl{opt_model.N} = [zeros(n_obs*n_ineq+n_obs*n_ineq,1); d_min*ones(n_obs,1); -inf*ones(n_obs,1); 0];
    
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
    
    opt_model.xinitidx = [1:n_x, n_x+n_obs*n_ineq+n_obs*n_ineq+n_u+1:n_x+n_obs*n_ineq+n_obs*n_ineq+n_u+n_u];
    
    opt_codeopts = getOptions(params.name);
    opt_codeopts.maxit = 300;
    opt_codeopts.printlevel = 2;
    opt_codeopts.optlevel = 0;
    opt_codeopts.nlp.ad_tool = 'casadi-351';
    opt_codeopts.nlp.BuildSimulinkBlock = 0;

    FORCES_NLP(opt_model, opt_codeopts);
end

% Stage cost
function opt_obj = eval_opt_obj(z, p)
    global n_obs n_ineq n_x n_u Q R
    
    x = z(1:n_x);
    u = z(n_x+n_obs*n_ineq+n_obs*n_ineq+1:n_x+n_obs*n_ineq+n_obs*n_ineq+n_u);
    x_ref = p(1:n_x);
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref) + bilin(R, u, u);
end

% Terminal cost
function opt_obj = eval_opt_obj_N(z, p)
    global n_x Q
    
    x = z(1:n_x);
    x_ref = p(1:n_x);
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref);
end

function opt_eq = eval_opt_eq(z, p)
    global n_x n_u n_obs n_ineq d_ineq G dynamics
    
    x = z(1:n_x);
    u = z(n_x+n_obs*n_ineq+n_obs*n_ineq+1:n_x+n_obs*n_ineq+n_obs*n_ineq+n_u);
    
    R_opt = [cos(z(3)), -sin(z(3)); sin(z(3)), cos(z(3))];

    opt_eq = dynamics.f_dt_aug(x, u);
    for i = 1:n_obs
        A = reshape(p(n_x+(i-1)*n_ineq*d_ineq+1:n_x+i*n_ineq*d_ineq), n_ineq, d_ineq);
        lambda = z((i-1)*n_ineq+1:i*n_ineq);
        mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);

        opt_eq = vertcat(opt_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_opt)), lambda));
    end
end

function opt_eq = eval_opt_eq_Nm1(z, p)
    global n_x n_u n_obs n_ineq d_ineq G dynamics
    
    x = z(1:n_x);
    u = z(n_x+n_obs*n_ineq+n_obs*n_ineq+1:n_x+n_obs*n_ineq+n_obs*n_ineq+n_u);
    
    R_opt = [cos(z(3)), -sin(z(3)); sin(z(3)), cos(z(3))];

    opt_eq = dynamics.f_dt(x, u);
    for i = 1:n_obs
        A = reshape(p(n_x+(i-1)*n_ineq*d_ineq+1:n_x+i*n_ineq*d_ineq), n_ineq, d_ineq);
        lambda = z((i-1)*n_ineq+1:i*n_ineq);
        mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);
        
        opt_eq = vertcat(opt_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_opt)), lambda));
    end
end

function opt_ineq = eval_opt_ineq(z, p)
    global n_x n_u n_obs n_ineq d_ineq g

    x = z(1:n_x);
    u = z(n_x+n_obs*n_ineq+n_obs*n_ineq+1:n_x+n_obs*n_ineq+n_obs*n_ineq+n_u);
    u_p = z(n_x+n_obs*n_ineq+n_obs*n_ineq+n_u+1:n_x+n_obs*n_ineq+n_obs*n_ineq+n_u+n_u);
    hyp_w = p(n_x+n_obs*n_ineq*d_ineq+n_obs*n_ineq+1:n_x+n_obs*n_ineq*d_ineq+n_obs*n_ineq+n_x);
    hyp_b = p(n_x+n_obs*n_ineq*d_ineq+n_obs*n_ineq+n_x+1);
    
    t_opt = z(1:2);
    
    opt_ineq = [];
    for i = 1:n_obs
        A = reshape(p(n_x+(i-1)*n_ineq*d_ineq+1:n_x+i*n_ineq*d_ineq), n_ineq, d_ineq);
        b = p(n_x+n_obs*n_ineq*d_ineq+(i-1)*n_ineq+1:n_x+n_obs*n_ineq*d_ineq+i*n_ineq);    
        lambda = z((i-1)*n_ineq+1:i*n_ineq);
        mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);
        
        opt_ineq = vertcat(opt_ineq, lambda);
        opt_ineq = vertcat(opt_ineq, mu);
        opt_ineq = vertcat(opt_ineq, -dot(g, mu)+mtimes(transpose(mtimes(A,t_opt)-b),lambda));
        opt_ineq = vertcat(opt_ineq, dot(mtimes(transpose(A), lambda), mtimes(transpose(A),lambda)));     
    end

    opt_ineq = vertcat(opt_ineq, dot(hyp_w, x) - hyp_b);
    opt_ineq = vertcat(opt_ineq, u);
    opt_ineq = vertcat(opt_ineq, u-u_p);
end

function opt_ineq = eval_opt_ineq_N(z, p)
    global n_x n_obs n_ineq d_ineq g
    
    x = z(1:n_x);
    hyp_w = p(n_x+n_obs*n_ineq*d_ineq+n_obs*n_ineq+1:n_x+n_obs*n_ineq*d_ineq+n_obs*n_ineq+n_x);
    hyp_b = p(n_x+n_obs*n_ineq*d_ineq+n_obs*n_ineq+n_x+1);
    
    t_opt = z(1:2);
    
    opt_ineq = [];
    for i = 1:n_obs
        A = reshape(p(n_x+(i-1)*n_ineq*d_ineq+1:n_x+i*n_ineq*d_ineq), n_ineq, d_ineq);
        b = p(n_x+n_obs*n_ineq*d_ineq+(i-1)*n_ineq+1:n_x+n_obs*n_ineq*d_ineq+i*n_ineq);
        lambda = z((i-1)*n_ineq+1:i*n_ineq);
        mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);
        
        opt_ineq = vertcat(opt_ineq, lambda);
        opt_ineq = vertcat(opt_ineq, mu);
        opt_ineq = vertcat(opt_ineq, -dot(g, mu)+mtimes(transpose(mtimes(A,t_opt)-b),lambda));
        opt_ineq = vertcat(opt_ineq, dot(mtimes(transpose(A), lambda), mtimes(transpose(A),lambda)));
    end
    
    opt_ineq = vertcat(opt_ineq, dot(hyp_w, x) - hyp_b);
end