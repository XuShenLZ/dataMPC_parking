%% plotExp: plot the experiment process
function F = plotExp(dataname, plt_params)
    close all;
    
	load(dataname)
    
    if ~isfield(exp_params, 'map_dim')
        map_dim = [-30 30 -10 10];
    else
        map_dim = exp_params.map_dim;
    end
    colors = {'#0072BD', '#D95319', '#77AC30'};
    
	T = exp_params.T;
	N = exp_params.controller.N;
	r = sqrt(OEV.width^2 + OEV.length^2)/2; % Collision buffer radius
	d_lim = exp_params.controller.d_lim;
	a_lim = exp_params.controller.a_lim;
    
    % Default plot parameter values
    if isfield(plt_params, 'plt_hyp')
        plt_hyp = plt_params.plt_hyp;
    else
        plt_hyp = false;
    end
    if isfield(plt_params, 'plt_ref')
        plt_ref = plt_params.plt_ref;
    else
        plt_ref = false;
    end
    if isfield(plt_params, 'plt_sol')
        plt_sol = plt_params.plt_sol;
    else
        plt_sol = false;
    end
    if isfield(plt_params, 'plt_preds')
        plt_preds = plt_params.plt_preds;
    else
        plt_preds = true;
    end
    if isfield(plt_params, 'plt_tv_preds')
        plt_tv_preds = plt_params.plt_tv_preds;
    else
        plt_tv_preds = false;
    end
    if isfield(plt_params, 'plt_col_buf')
        plt_col_buf = plt_params.plt_col_buf;
    else
        plt_col_buf = false;
    end
    
    OEV_plt_opts.circle = false;
	OEV_plt_opts.frame = false;
	OEV_plt_opts.color = 'g';
	OEV_plt_opts.alpha = 0.5;
    
    if plt_params.plt_col_buf
        EV_plt_opts.circle = true;
    else
        EV_plt_opts.circle = false;
    end
	EV_plt_opts.frame = true;
	EV_plt_opts.color = 'b';
	EV_plt_opts.alpha = 0.5;

	TV_plt_opts.color = 'y';

	cmap = jet(N+1);
    
    l_s = []; t_s = [];
	p_EV = []; l_EV = [];
	p_OEV = []; l_OEV = [];
	p_TV = []; l_TV = [];
	t_EV_ref = [];
	t_Y = [];
	t_h_feas = []; t_h_safe = [];

	rot = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

	F(T-N) = struct('cdata',[],'colormap',[]);

	fig = figure('Position', [50 50 1200 600], 'Visible', plt_params.visible);

	ax1 = axes('Position',[0.05 0.6 0.4 0.4]);
    if isfield(exp_params, 'lane_width')
        yline(exp_params.lane_width/2, '-.', 'color', '#7E2F8E', 'linewidth', 2)
        hold on
        yline(-exp_params.lane_width/2, '-.', 'color', '#7E2F8E', 'linewidth', 2)
    end
	axis equal
	axis(map_dim);
    if isfield(exp_params, 'name')
        title(sprintf('%s, exp %i', exp_params.name, exp_params.exp_num))
    end

    if isfield(exp_params, 'strategy_names')
        strategy_names = exp_params.strategy_names;
    else
        strategy_names = ["Left", "Right", "Yield"];
    end

    ax_state = axes('Position',[0.1 0.05 0.35 0.4]);
    state_line = animatedline(ax_state, 'color', colors{1}, 'linewidth', 2);
    h_c_l = animatedline(ax_state, 'color', colors{2}, 'linewidth', 2, 'linestyle', '--');
    legend('State', 'Collision')
    ylabel('States')
    yticks(0:6)
    yticklabels({'Free-Driving', ...
        'Safe-Confidence', 'Safe-Yield', 'Safe-Infeasible', ...
        'HOBCA-Unlocked', 'HOBCA-Locked', ...
        'Emergency-Break'})
    axis([1 T-N 0 6])
    grid on

	% Plot inputs
	ax_h_v = axes('Position',[0.5 0.75 0.45 0.2]);
	h_v_l = animatedline(ax_h_v, 'color', '#0072BD', 'linewidth', 2);
	ylabel('v')
	axis([1 T-N -3 3])

	ax_h_d = axes('Position',[0.5 0.52 0.45 0.2]);
	h_d_l = animatedline(ax_h_d, 'color', '#0072BD', 'linewidth', 2);
	ylabel('delta')
	axis([1 T-N d_lim(1) d_lim(2)])

	ax_h_a = axes('Position',[0.5 0.28 0.45 0.2]);
	h_a_l = animatedline(ax_h_a, 'color', '#0072BD', 'linewidth', 2);
	ylabel('a')
	axis([1 T-N a_lim(1) a_lim(2)])

	ax_score = axes('Position',[0.5 0.05 0.45 0.2]);
	L_line = animatedline(ax_score, 'color', colors{1}, 'linewidth', 2);
	R_line = animatedline(ax_score, 'color', colors{2}, 'linewidth', 2);
	Y_line = animatedline(ax_score, 'color', colors{3}, 'linewidth', 2);
	legend('Left', 'Right', 'Yield', 'Location', 'southeast', 'Orientation', 'horizontal')
	prob_dim = [1 T-N 0 1];
	ylabel('Score')
	axis(prob_dim)
	grid on
    
    % Placeholder for plotting collision boundary
    phi = linspace(0, 2*pi, 200);
    coll_bound_x = zeros(1, 200);
    coll_bound_y = zeros(1, 200);
                    
    for i = 1:T-N
        % Delete lines and patches from last iteration
	    delete(p_EV); delete(l_EV); delete(p_OEV); delete(l_OEV); 
	    delete(p_TV); delete(l_TV); 
	    delete(t_EV_ref); delete(t_Y); delete(t_h_feas); delete(t_h_safe);
        
	    z_ref = z_refs(:,:,i);
	    z_pred = z_preds(:,:,i);
	    u_pred = u_preds(:,:,i);
	    
        if z_traj(1,i) > map_dim(2)
	        F(i:end) = [];
	        break;
        end
	    
        if exist('scores', 'var')
            delete(l_s); delete(t_s);
            axes(ax_score);
            hold on
            score = scores(:,i);
            addpoints(L_line, i, score(1));
            addpoints(R_line, i, score(2));
            addpoints(Y_line, i, score(3));
            
            l_s = plot(i, max(score), 'ko', 'DisplayName', 'Max');
            hold on
            t_s = text(i, max(score)+0.05, sprintf('%g', max(score)));
        end
	    
	    addpoints(h_v_l, i, z_traj(4,i));
	    addpoints(h_d_l, i, u_traj(1,i));
	    addpoints(h_a_l, i, u_traj(2,i));

        if exist('collide', 'var')
            addpoints(h_c_l, i, double(collide(i)));
        end


        if exist('ebrake', 'var') && exist('safety', 'var')
            if ebrake(i)
                addpoints(state_line, i, 6); % Emergency-Break
            elseif safety(i)
                addpoints(state_line, i, 1); % Use Safety-Confidence    
            end
        elseif exist('FSM_states', 'var')
            switch FSM_states{i}
                case "Free-Driving"
                    addpoints(state_line, i, 0);
                case "Safe-Confidence"
                    addpoints(state_line, i, 1);
                case "Safe-Yield"
                    addpoints(state_line, i, 2);
                case "Safe-Infeasible"
                    addpoints(state_line, i, 3);
                case "HOBCA-Unlocked"
                    addpoints(state_line, i, 4);
                case "HOBCA-Locked"
                    addpoints(state_line, i, 5);
                case "Emergency-Break"
                    addpoints(state_line, i, 6);
                otherwise
                    error('Ploting FSM states: Undefined State');
            end
        end

	    % Plot
	    axes(ax1);
        if exist('strategy_idxs', 'var')
            t_Y = text(ax1, -29, 9, sprintf('Strategy: %s; Lock: %d', strategy_names(strategy_idxs(i)), strategy_locks(i)), 'color', 'k');
            hold on
        elseif exist('FSM_states', 'var') && exist('FSM_states', 'var')
            t_Y = text(ax1, -29, 9, sprintf('Operation State: %s; Strategy: %s', FSM_states{i}, strategies{i}), 'color', 'k');
        else
            t_Y = [];
        end
        
        if exist('safety', 'var')
            if safety(i)
                s_h = 'ON';
                ws_stat = 'n/a';
                sol_stat = 'n/a';
            else
                s_h = 'OFF';
                ws_stat = 'n/a';
                sol_stat = sol_stats{i}.return_status;
                if exist('ws_stats', 'var')
                    ws_stat = ws_stats{i}.return_status;
                    if ~ws_stats{i}.success
                        sol_stat = 'n/a';
                    end
                end
            end
        elseif exist('FSM_states', 'var')
            if any(FSM_states{i} == ["Safe-Confidence", "Safe-Yield", "Safe-Infeasible"])
                ws_stat = 'n/a';
                sol_stat = 'n/a';
            else
                ws_stat = 'n/a';
                sol_stat = sol_stats{i}.return_status;
                if exist('ws_stats', 'var')
                    ws_stat = ws_stats{i}.return_status;
                    if ~ws_stats{i}.success
                        sol_stat = 'n/a';
                    end
                end
            end
        else
            s_h = 'n/a';
            ws_stat = 'n/a';
            sol_stat = sol_stats{i}.return_status;
            if exist('ws_stats', 'var')
                ws_stat = ws_stats{i}.return_status;
                if ~ws_stats{i}.success
                    sol_stat = 'n/a';
                end
            end
        end
        
        if exist('ebrake', 'var')
            if ebrake(i)
    	        e_h = 'ON';
    	    else
    	        e_h = 'OFF';
            end
        end
        
	    t_h_feas = text(0.02, 0.85, sprintf('HOBCA Online MPC (ws): %s, (sol): %s', ws_stat, sol_stat), 'color', 'b', 'interpreter', 'none', 'Units', 'normalized');

        if exist('ebrake', 'var') && exist('safety', 'var')
    	    t_h_safe = text(0.02, 0.9, sprintf('Safety: %s, E-Brake: %s', s_h, e_h), 'color', 'b', 'Units', 'normalized');
        else
            t_h_safe = [];
        end
        
        if plt_sol
            [p_OEV, l_OEV] = plotCar(OEV.traj(1,i), OEV.traj(2,i), OEV.traj(3,i), OEV.width, OEV.length, OEV_plt_opts);
        end
        [p_EV, l_EV] = plotCar(z_traj(1,i), z_traj(2,i), z_traj(3,i), OEV.width, OEV.length, EV_plt_opts);
        
	    p_TV = [];
	    l_TV = [];
        
        % Plot TV over horizon and predictions
        for j = 1:N+1
            if j == 1
	            TV_plt_opts.alpha = 0.5;
	            TV_plt_opts.frame = true;
                if plt_col_buf
                    TV_plt_opts.circle = true;
                else
                    TV_plt_opts.circle = false;
                end
                
                [p, l] = plotCar(TV.x(i+j-1), TV.y(i+j-1), TV.heading(i+j-1), TV.width, TV.length, TV_plt_opts);
                p_TV = [p_TV, p];
                l_TV = [l_TV, l];
            elseif plt_tv_preds
                TV_plt_opts.circle = false;
	            TV_plt_opts.alpha = 0;
	            TV_plt_opts.frame = false;
                
                [p, l] = plotCar(TV.x(i+j-1), TV.y(i+j-1), TV.heading(i+j-1), TV.width, TV.length, TV_plt_opts);
                p_TV = [p_TV, p];
                l_TV = [l_TV, l];
            end
	        
            if plt_hyp && exist('hyps', 'var')
                hyp = hyps{i};
                if ~isnan(hyp{j}.pos)
                    for k = 1:length(phi)
                        if exist('vel_dirs', 'var') && exist('scalings', 'var')
                            [x_b, y_b, ~, ~] = get_collision_boundary_point_tight_inflated(0, 0, phi(k), TV.width, TV.length, r, vel_dirs(1,j,i), scalings(1,j,i));
                        else
                            [x_b, y_b, ~, ~] = get_collision_boundary_point_tight(0, 0, phi(k), TV.width, TV.length, r);
                        end
                        coll_bound_x(k) = x_b;
                        coll_bound_y(k) = y_b;
                    end
    
                    coll_bound_global = rot(TV.heading(i+j-1))*[coll_bound_x; coll_bound_y] + [TV.x(i+j-1); TV.y(i+j-1)];
                    l_TV = [l_TV plot(coll_bound_global(1,:), coll_bound_global(2,:), 'color', cmap(j,:))];
                    if hyp{j}.w(2) == 0
                        hyp_x = [hyp{j}.b, hyp{j}.b];
                        hyp_y = [map_dim(3), map_dim(4)];
                    else
                        hyp_x = [map_dim(1), map_dim(2)];
                        hyp_y = (-hyp{j}.w(1)*hyp_x+hyp{j}.b)/hyp{j}.w(2);
                    end
                    l_TV = [l_TV plot(hyp_x, hyp_y, 'color', cmap(j,:))];
                    l_TV = [l_TV plot([z_ref(1,j) hyp{j}.pos(1)], [z_ref(2,j) hyp{j}.pos(2)], '-o', 'color', cmap(j,:))];
                end
            end
            
            if plt_ref
                l_TV = [l_TV plot(z_ref(1,j), z_ref(2,j), '.', 'color', cmap(j,:), 'markersize', 10)];
            end
            
            if plt_preds
                if exist('safety', 'var')
                    if ~safety(i)
                        l_TV = [l_TV plot(z_pred(1,j), z_pred(2,j), 'd', 'color', cmap(j,:))];
                    end
                else
                    l_TV = [l_TV plot(z_pred(1,j), z_pred(2,j), 'd', 'color', cmap(j,:))];
                end
            end
        end
        
	    axis equal
	    axis(map_dim);

	    axes(ax_score)
	    axis auto
	    axis(prob_dim);
        
	    drawnow

	    F(i) = getframe(fig);
    end

    if plt_params.mv_save
        
        if isfield(plt_params, 'mv_path')
            if ~isfolder(plt_params.mv_path)
                mkdir(plt_params.mv_path)
            end
            file = sprintf('%s.mp4', plt_params.mv_name);
            path = plt_params.mv_path;
        else
            % Save Movie
            if ~isfolder('../../movies/')
                mkdir('../../movies')
            end

            [file,path] = uiputfile(sprintf('../../movies/%s.mp4', plt_params.mv_name));
        end

		v = VideoWriter([path, file], 'MPEG-4');
		v.FrameRate = 10;

		try
			open(v);
			writeVideo(v,F);
			close(v);

			fprintf('Video saved as %s\n', [path, file])
		catch
			disp('Error in saving video')
		end
    end
end