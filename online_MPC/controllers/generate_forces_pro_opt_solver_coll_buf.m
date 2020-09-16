function generate_forces_pro_opt_solver_coll_buf(params)
    global n_x n_u n_obs Q R dynamics EV_r
    
    n_x = params.n_x;
    n_u = params.n_u;
    n_obs = params.n_obs;
    Q = params.Q;
    R = params.R;
    dt = params.dt;
    dynamics = params.dynamics;
    u_l = params.u_l;
    u_u = params.u_u;
    du_l = params.du_l;
    du_u = params.du_u;
    EV_r = params.EV_r;
    
    opt_model = {};
    
    opt_model.N = params.N + 1;
    
    nvar = zeros(opt_model.N,1); 
    neq = zeros(opt_model.N-1,1); 
    nh = zeros(opt_model.N,1); 
    npar = zeros(opt_model.N,1); 
    objective = cell(opt_model.N,1);
    ls_objective = cell(opt_model.N,1);
    eq = cell(opt_model.N-1,1);
    E = cell(opt_model.N-1,1);
    ineq = cell(opt_model.N,1);
    hu = cell(opt_model.N,1); 
    hl = cell(opt_model.N,1);
    ub = cell(opt_model.N,1);
    lb = cell(opt_model.N,1);
    
    for i = 1:opt_model.N-1
        objective{i} = @eval_opt_obj;
        ls_objective{i} = @eval_opt_ls_obj;
        
        % [x_k, u_k, u_km1]
        nvar(i) = n_x + n_u + n_u;
        ub{i} = [inf*ones(n_x,1); u_u; u_u];
        lb{i} = [-inf*ones(n_x,1); u_l; u_l];
        
        % [coll_buf, du]
        nh(i) = n_obs + n_u;
        ineq{i} = @eval_opt_ineq;
        hu{i} = [inf*ones(n_obs,1); dt*du_u];
        hl{i} = [zeros(n_obs,1); dt*du_l];
        
        % [x_ref, obs_xy, obs_r]
        npar(i) = n_x + n_obs*2 + n_obs;
        
        if i == opt_model.N-1
            % [dynamics]
            neq(i) = n_x;
            E{i} = [eye(n_x)];
            eq{i} = @eval_opt_eq_Nm1;
        else
            % [augmented dynamics]
            neq(i) = n_x + n_u;
            E{i} = [eye(n_x), zeros(n_x, n_u + n_u);
                zeros(n_u,  n_x + n_u), eye(n_u)];
            eq{i} = @eval_opt_eq;
        end
    end
    objective{opt_model.N} = @eval_opt_obj_N;
    ls_objective{opt_model.N} = @eval_opt_ls_obj_N;
    
    % [x_k]
    nvar(opt_model.N) = n_x;
    ub{opt_model.N} = [inf*ones(n_x,1)];
    lb{opt_model.N} = [-inf*ones(n_x,1)];
    
    % [coll_buf]
    nh(opt_model.N) = n_obs;
    ineq{opt_model.N} = @eval_opt_ineq_N;
    hu{opt_model.N} = [inf*ones(n_obs,1)];
    hl{opt_model.N} = [zeros(n_obs,1)];
    
    % [x_ref, obs_xy, obs_r]
    npar(opt_model.N) = n_x + n_obs*2 + n_obs;
    
    opt_model.nvar = nvar;
    opt_model.neq = neq;
    opt_model.nh = nh;
    opt_model.npar = npar;

    opt_model.objective = objective;
    opt_model.LSobjective = ls_objective;
    opt_model.eq = eq;
    opt_model.E = E;
    opt_model.ineq = ineq;
    opt_model.hu = hu;
    opt_model.hl = hl;
    opt_model.ub = ub;
    opt_model.lb = lb;
    
    opt_model.xinitidx = [1:n_x, n_x+n_u+1:n_x+n_u+n_u];
    
    opt_codeopts = getOptions(params.name);
    opt_codeopts.overwrite = 1;
    opt_codeopts.printlevel = 2;
    opt_codeopts.optlevel = 3;
    opt_codeopts.BuildSimulinkBlock = 0;

    opt_codeopts.nlp.ad_tool = 'casadi-351';
    opt_codeopts.nlp.linear_solver = 'symm_indefinite';

    opt_codeopts.nlp.TolStat = 1e-3;
    opt_codeopts.nlp.TolEq = 1e-3;
    opt_codeopts.nlp.TolIneq = 1e-3;

    FORCES_NLP(opt_model, opt_codeopts);
end

% Stage cost
function opt_obj = eval_opt_obj(z, p)
    global n_x n_u Q R
    
    x = z(1:n_x);
    u = z(n_x+1:n_x+n_u);
    x_ref = p(1:n_x);
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref) + bilin(R, u, u);
end

function opt_obj = eval_opt_ls_obj(z, p)
    global n_x n_u Q R
    
    x = z(1:n_x);
    u = z(n_x+1:n_x+n_u);
    x_ref = p(1:n_x);
    
    opt_obj = mtimes(sqrtm(blkdiag(Q,R)), vertcat(x-x_ref,u));
end

% Terminal cost
function opt_obj = eval_opt_obj_N(z, p)
    global n_x Q
    
    x = z(1:n_x);
    x_ref = p(1:n_x);
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref);
end

function opt_obj = eval_opt_ls_obj_N(z, p)
    global n_x Q
    
    x = z(1:n_x);
    x_ref = p(1:n_x);
    
    opt_obj = mtimes(sqrtm(Q), x-x_ref);
end

% Equality constraints at each stage
function opt_eq = eval_opt_eq(z, p)
    global n_x n_u dynamics
    
    x = z(1:n_x);
    u = z(n_x+1:n_x+n_u);

    opt_eq = dynamics.f_dt_aug(x, u);
end

% Equality constraints at last stage
function opt_eq = eval_opt_eq_Nm1(z, p)
    global n_x n_u dynamics
    
    x = z(1:n_x);
    u = z(n_x+1:n_x+n_u);

    opt_eq = dynamics.f_dt(x, u);
end

% Inequality constraints at each stage
function opt_ineq = eval_opt_ineq(z, p)
    global n_x n_u n_obs EV_r

    x = z(1:n_x);
    u = z(n_x+1:n_x+n_u);
    u_p = z(n_x+n_u+1:n_x+n_u+n_u);
    
    opt_ineq = [];
    for i = 1:n_obs
        pos = reshape(p(n_x+(i-1)*2+1:n_x+i*2), 2, 1);
        r = p(n_x+n_obs*2+i);    

        opt_ineq = vertcat(opt_ineq, dot(x(1:2)-pos, x(1:2)-pos)-(r+EV_r)^2);     
    end
    
    opt_ineq = vertcat(opt_ineq, u-u_p);
end

% Inequality constraints at last stage
function opt_ineq = eval_opt_ineq_N(z, p)
    global n_x n_obs EV_r
    
    x = z(1:n_x);
    
    opt_ineq = [];
    for i = 1:n_obs
        pos = reshape(p(n_x+(i-1)*2+1:n_x+i*2), 2, 1);
        r = p(n_x+n_obs*2+i);  
        
        opt_ineq = vertcat(opt_ineq, dot(x(1:2)-pos, x(1:2)-pos)-(r+EV_r)^2);  
    end
end