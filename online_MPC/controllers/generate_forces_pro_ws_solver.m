function generate_forces_pro_ws_solver(params) 
    global n_x n_obs n_ineq d_ineq m_ineq N_ineq M_ineq G g 
    
    n_x = params.n_x;
    n_obs = params.n_obs;
    n_ineq = params.n_ineq; % Number of constraints for each obstacle
    d_ineq = params.d_ineq; % Dimension of constraints for all obstacles
    G = params.G;
    g = params.g;
    m_ineq = size(G,1); % Number of constraints for controlled object
    
    N_ineq = sum(n_ineq);
    M_ineq = n_obs*m_ineq;
    
    ws_model = {};
    
    ws_model.N = params.N + 1;
    
    ws_model.objective = @eval_ws_obj;
    ws_model.objectiveN = @(z) 0;
    
    % [lambda, mu, d]
    ws_model.nvar = N_ineq + M_ineq + n_obs;
    ws_model.ub = [inf*ones(N_ineq+M_ineq,1); inf*ones(n_obs,1)];
    ws_model.lb = [zeros(N_ineq+M_ineq,1); -inf*ones(n_obs,1)];
    
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
    ws_model.npar = n_x + N_ineq*d_ineq + N_ineq;

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
    global N_ineq M_ineq
    
    d = z(N_ineq+M_ineq+1:end);
    ws_obj = -sum(d);
end
    
function ws_eq = eval_ws_eq(z, p)
    global n_x n_obs n_ineq d_ineq m_ineq N_ineq M_ineq G g 
    
    t_ws = p(1:2);
    R_ws = [cos(p(3)), -sin(p(3)); sin(p(3)), cos(p(3))];

    ws_eq = [];
    j = 0;
    for i = 1:n_obs
        A = reshape(p(n_x+j*d_ineq+1:n_x+j*d_ineq+n_ineq(i)*d_ineq), n_ineq(i), d_ineq);
        b = p(n_x+N_ineq*d_ineq+j+1:n_x+N_ineq*d_ineq+j+n_ineq(i));
        lambda = z(j+1:j+n_ineq(i));
        mu = z(N_ineq+(i-1)*m_ineq+1:N_ineq+i*m_ineq);
        d = z(N_ineq+M_ineq+i);
        ws_eq = vertcat(ws_eq, -dot(g, mu)+dot(mtimes(A, t_ws)-b, lambda)-d);
        ws_eq = vertcat(ws_eq, mtimes(G', mu)+mtimes(transpose(mtimes(A, R_ws)), lambda));
        
        j = j + n_ineq(i);
    end
end

function ws_ineq = eval_ws_ineq(z, p)
    global n_x n_obs n_ineq d_ineq
    
    ws_ineq = [];
    j = 0;
    for i = 1:n_obs
        A = reshape(p(n_x+j*d_ineq+1:n_x+j*d_ineq+n_ineq(i)*d_ineq), n_ineq(i), d_ineq);
        lambda = z(j+1:j+n_ineq(i));
        ws_ineq = vertcat(ws_ineq, dot(mtimes(transpose(A),lambda), mtimes(transpose(A),lambda)));
        
        j = j + n_ineq(i);
    end
end