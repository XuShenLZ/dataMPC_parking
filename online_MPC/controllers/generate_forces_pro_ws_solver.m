function generate_forces_pro_ws_solver(params) 
    global n_x n_obs n_ineq d_ineq G g 
    
    n_x = params.n_x;
    n_obs = params.n_obs;
    n_ineq = params.n_ineq;
    d_ineq = params.d_ineq;
    G = params.G;
    g = params.g;
    
    ws_model = {};
    
    ws_model.N = params.N + 1;
    
    ws_model.objective = @eval_ws_obj;
    ws_model.objectiveN = @(z) 0;
    
    % [lambda, mu, d]
    ws_model.nvar = n_obs*n_ineq + n_obs*n_ineq + n_obs;
    ws_model.ub = [inf*ones(n_obs*n_ineq+n_obs*n_ineq,1); inf*ones(n_obs,1)];
    ws_model.lb = [zeros(n_obs*n_ineq+n_obs*n_ineq,1); -inf*ones(n_obs,1)];
    
    % [obca dist, obca]
    ws_model.neq = n_obs + n_obs*d_ineq;
    ws_model.eq = @eval_ws_eq;
    ws_model.E = zeros(ws_model.neq, ws_model.nvar);
    
    % [obca norm]
    ws_model.nh = n_obs;
    ws_model.ineq = @eval_ws_ineq;
    ws_model.hu = ones(n_obs,1);
    ws_model.hl = -inf*ones(n_obs,1);
    
    % [x_ref, obs_A, obs_b]
    ws_model.npar = n_x + n_obs*n_ineq*d_ineq + n_obs*n_ineq;

    ws_codeopts = getOptions(params.name);
%     ws_codeopts.maxit = 1000;
    ws_codeopts.overwrite = 1;
    ws_codeopts.printlevel = 2;
    ws_codeopts.optlevel = 3;
    ws_codeopts.BuildSimulinkBlock = 0;
    ws_codeopts.nlp.linear_solver = 'symm_indefinite';
    ws_codeopts.nlp.ad_tool = 'casadi-351';

    FORCES_NLP(ws_model, ws_codeopts);
end

function ws_obj = eval_ws_obj(z)
    global n_obs n_ineq
    
    d = z(n_obs*n_ineq+n_obs*n_ineq+1:end);
    ws_obj = -sum(d);
end
    
function ws_eq = eval_ws_eq(z, p)
    global n_x n_obs n_ineq d_ineq G g 
    
    t_ws = p(1:2);
    R_ws = [cos(p(3)), -sin(p(3)); sin(p(3)), cos(p(3))];

    ws_eq = [];
    for i = 1:n_obs
        A = reshape(p(n_x+(i-1)*n_ineq*d_ineq+1:n_x+i*n_ineq*d_ineq), n_ineq, d_ineq);
        b = p(n_x+n_obs*n_ineq*d_ineq+(i-1)*n_ineq+1:n_x+n_obs*n_ineq*d_ineq+i*n_ineq);
        lambda = z((i-1)*n_ineq+1:i*n_ineq);
        mu = z(n_obs*n_ineq+(i-1)*n_ineq+1:n_obs*n_ineq+i*n_ineq);
        d = z(n_obs*n_ineq+n_obs*n_ineq+i);
        ws_eq = vertcat(ws_eq, -dot(g, mu)+dot(mtimes(A, t_ws)-b, lambda)-d);
        ws_eq = vertcat(ws_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_ws)), lambda));
    end
end

function ws_ineq = eval_ws_ineq(z, p)
    global n_x n_obs n_ineq d_ineq
    
    ws_ineq = [];
    for i = 1:n_obs
        A = reshape(p(n_x+(i-1)*n_ineq*d_ineq+1:n_x+i*n_ineq*d_ineq), n_ineq, d_ineq);
        lambda = z((i-1)*n_ineq+1:i*n_ineq);
        ws_ineq = vertcat(ws_ineq, dot(mtimes(transpose(A),lambda), mtimes(transpose(A),lambda)));
    end
end