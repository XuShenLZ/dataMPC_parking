function generate_forces_pro_tracking_solver(params)
    global n_x n_u dynamics
    
    n_x = params.n_x;
    n_u = params.n_u;
    dynamics = params.dynamics;
    
    opt_model = {};
    
    opt_model.N = params.N + 1;
    
    nvar = zeros(opt_model.N,1); 
    neq = zeros(opt_model.N-1,1); 
%     nh = zeros(opt_model.N,1); 
    npar = zeros(opt_model.N,1); 
    objective = cell(opt_model.N,1);
    eq = cell(opt_model.N-1,1);
    E = cell(opt_model.N-1,1);
%     ineq = cell(opt_model.N,1);
%     hu = cell(opt_model.N,1); 
%     hl = cell(opt_model.N,1);
%     ub = cell(opt_model.N,1);
%     lb = cell(opt_model.N,1);
    ubidx = cell(opt_model.N,1);
    lbidx = cell(opt_model.N,1);
    
    for i = 1:opt_model.N-1
        objective{i} = @eval_opt_obj;
        
        % [x_k, u_k, u_km1]
        nvar(i) = n_x + n_u + n_u;
%         ub{i} = [10; 10; 10; 1; u_u; u_u];
%         lb{i} = [-10; -10; -10; -1; u_l; u_l];
        ubidx{i} = 1:nvar(i);
        lbidx{i} = 1:nvar(i);
        
        % [x_ref, diag(Q), diag(R), diag(R_d)]
        npar(i) = n_x + n_x + n_u + n_u;
        
        if i == opt_model.N-1
            % [dynamics]
            neq(i) = n_x;
            E{i} = eye(n_x);
            eq{i} = @eval_opt_eq_Nm1;
        else
            % [augmented dynamics]
            neq(i) = n_x + n_u;
            E{i} = [eye(n_x), zeros(n_x, n_u + n_u);
                    zeros(n_u, n_x + n_u), eye(n_u)];
            eq{i} = @eval_opt_eq;
        end
    end
    objective{opt_model.N} = @eval_opt_obj_N;
    
    % [x_k]
    nvar(opt_model.N) = n_x;
%     ub{opt_model.N} = [10; 10; 10; 1];
%     lb{opt_model.N} = [-10; -10; -10; -1];
    ubidx{opt_model.N} = 1:nvar(opt_model.N);
    lbidx{opt_model.N} = 1:nvar(opt_model.N);
    
    % [x_ref, diag(Q)]
    npar(opt_model.N) = n_x + n_x;
    
    opt_model.nvar = nvar;
    opt_model.neq = neq;
    opt_model.npar = npar;

    opt_model.objective = objective;
    opt_model.eq = eq;
    opt_model.E = E;
%     opt_model.ub = ub;
%     opt_model.lb = lb;
    opt_model.ub = [];
    opt_model.lb = [];
    opt_model.ubidx = ubidx;
    opt_model.lbidx = lbidx;
    opt_model.xinitidx = [1:n_x, n_x+n_u+1:n_x+n_u+n_u];
    
    opt_codeopts = getOptions(params.name);
    opt_codeopts.overwrite = 1;
    opt_codeopts.printlevel = 1;
    opt_codeopts.optlevel = params.optlevel;
    opt_codeopts.BuildSimulinkBlock = 0;
    opt_codeopts.cleanup = 0;
    opt_codeopts.platform = 'Generic';
    opt_codeopts.gnu = 1;
    opt_codeopts.sse = 1;
    
    opt_codeopts.nlp.ad_tool = 'casadi-351';
    opt_codeopts.nlp.linear_solver = 'symm_indefinite';
    opt_codeopts.nlp.stack_parambounds = 1;

    opt_codeopts.nlp.TolStat = 1e-3;
    opt_codeopts.nlp.TolEq = 1e-3;
    opt_codeopts.nlp.TolIneq = 1e-3;
    opt_codeopts.accuracy.ineq = 1e-3;  % infinity norm of residual for inequalities
    opt_codeopts.accuracy.eq = 1e-3;    % infinity norm of residual for equalities
    opt_codeopts.accuracy.mu = 1e-3;    % absolute duality gap
    opt_codeopts.accuracy.rdgap = 1e-3; % relative duality gap := (pobj-dobj)/pobj
    
    if isfield(params, 'factor_aff')
        opt_codeopts.linesearch.factor_aff = params.factor_aff;
    end
    if isfield(params, 'factor_cc')
        opt_codeopts.linesearch.factor_cc = params.factor_cc;
    end
    if isfield(params, 'minstep')
        opt_codeopts.linesearch.minstep = params.minstep;
    end
    if isfield(params, 'maxstep')
        opt_codeopts.linesearch.maxstep = params.maxstep;
    end

    FORCES_NLP(opt_model, opt_codeopts);
end

% Stage cost
function opt_obj = eval_opt_obj(z, p)
    global n_x n_u
    
    x = z(1:n_x);
    u = z(n_x+1:n_x+n_u);
    u_km1 = z(n_x+n_u+1:n_x+n_u+n_u);
    
    x_ref = p(1:n_x);
    Q = diag(p(n_x+1:n_x+n_x));
    R = diag(p(n_x+n_x+1:n_x+n_x+n_u));
    R_d = diag(p(n_x+n_x+n_u+1:n_x+n_x+n_u+n_u));
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref) + bilin(R, u, u) + bilin(R_d, u-u_km1, u-u_km1);
end

% Terminal cost
function opt_obj = eval_opt_obj_N(z, p)
    global n_x
    
    x = z(1:n_x);
    
    x_ref = p(1:n_x);
    Q = diag(p(n_x+1:n_x+n_x));
    
    opt_obj = bilin(Q, x-x_ref, x-x_ref);
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
