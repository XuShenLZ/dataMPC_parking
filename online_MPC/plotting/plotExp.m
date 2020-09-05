%% plotExp: plot the experiment process
function F = plotExp(dataname, plt_params)

	load(dataname)

	map_dim = [-30 30 -10 10];
	strategies = ["Left", "Right", "Yield"];

	T = exp_params.T;
	N = exp_params.controller.N;
	r = sqrt(OEV.width^2 + OEV.length^2)/2; % Collision buffer radius
	d_lim = exp_params.controller.d_lim;
	a_lim = exp_params.controller.a_lim;

	OEV_plt_opts.circle = false;
	OEV_plt_opts.frame = false;
	OEV_plt_opts.color = 'g';
	OEV_plt_opts.alpha = 0.5;

	EV_plt_opts.circle = false;
	EV_plt_opts.frame = true;
	EV_plt_opts.color = 'b';
	EV_plt_opts.alpha = 0.5;

	TV_plt_opts.circle = false;
	TV_plt_opts.color = 'y';

	cmap = jet(N+1);

	p_EV = []; l_EV = [];
	p_OEV = []; l_OEV = [];
	p_TV = []; l_TV = [];
	t_EV_ref = [];
	t_Y = [];
	t_h_feas = []; t_h_safe = [];

	phi = linspace(0, 2*pi, 200);
	coll_bound_x = zeros(1, 200);
	coll_bound_y = zeros(1, 200);
	for i = 1:length(phi)
	    [x_b, y_b, ~, ~] = get_collision_boundary_point(0, 0, phi(i), TV.width, TV.length, r);
	    coll_bound_x(i) = x_b;
	    coll_bound_y(i) = y_b;
	end
	rot = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];

	F(T-N) = struct('cdata',[],'colormap',[]);

	fig = figure('Position', [50 50 1200 600], 'Visible', plt_params.visible);

	ax1 = axes('Position',[0.05 0.55 0.4 0.4]);
	yline(3.5, '-.', 'color', '#7E2F8E', 'linewidth', 2)
	hold on
	yline(-3.5, '-.', 'color', '#7E2F8E', 'linewidth', 2)
	axis equal
	axis(map_dim);

	ax2 = axes('Position',[0.05 0.05 0.4 0.4]);
	L_line = animatedline(ax2, 'color', '#0072BD', 'linewidth', 2);
	R_line = animatedline(ax2, 'color', '#D95319', 'linewidth', 2);
	Y_line = animatedline(ax2, 'color', '#77AC30', 'linewidth', 2);
	legend('Left', 'Right', 'Yield')
	prob_dim = [1 T-N 0 1];
	ylabel('Score')
	axis(prob_dim)
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

	ax_h_s = axes('Position',[0.5 0.05 0.45 0.2]);
	h_s_l = animatedline(ax_h_s, 'color', '#0072BD', 'linewidth', 2);
	h_e_l = animatedline(ax_h_s, 'color', '#D95319', 'linewidth', 2, 'linestyle', '--');
	legend('Safety', 'E-Brake')
	ylabel('safety')
	axis([1 T-N 0 1])

	for i = 1:T-N
	    status_ws = ws_stats{i};
	    status_sol = sol_stats{i};
	    hyp = hyps{i};
	    z_ref = z_refs(:,:,i);
	    z_pred = z_preds(:,:,i);
	    u_pred = u_preds(:,:,i);
	    score = scores(:,i);
	    
	    if z_traj(1,i) > map_dim(2)
	        F(i:end) = [];
	        break
	    end
	    
	    addpoints(L_line, i, score(1));
	    addpoints(R_line, i, score(2));
	    addpoints(Y_line, i, score(3));
	    
	    addpoints(h_v_l, i, z_traj(4,i));
	    addpoints(h_d_l, i, u_traj(1,i));
	    addpoints(h_a_l, i, u_traj(2,i));
	    addpoints(h_s_l, i, double(safety(i)));
	    addpoints(h_e_l, i, double(ebrake(i)));

	    % Delete lines and patches from last iteration
	    delete(p_EV); delete(l_EV); delete(p_OEV); delete(l_OEV); 
	    delete(p_TV); delete(l_TV); 
	    delete(t_EV_ref); delete(t_Y); delete(t_h_feas); delete(t_h_safe);
	    
	    % Plot
	    axes(ax1);
	    t_Y = text(-29, 9, sprintf('Strategy: %s; Lock: %d', strategies(strategy_idxs(i)), strategy_locks(i)), 'color', 'k');
	    hold on
	    
	    if safety(i)
	        s_h = 'ON';
	    else
	        s_h = 'OFF';
	    end
	    if ebrake(i)
	        e_h = 'ON';
	    else
	        e_h = 'OFF';
	    end
	    if safety(i)
	        ws_stat = 'n/a';
	        sol_stat = 'n/a';
	    else
	        ws_stat = status_ws.return_status;
	        if status_ws.success
	            sol_stat = status_sol.return_status;
	        else
	            sol_stat = 'n/a';
	        end
	    end
	    t_h_feas = text(-29, 7, sprintf('HOBCA Online MPC (ws): %s, (sol): %s', ws_stat, sol_stat), 'color', 'b', 'interpreter', 'none');
	    t_h_safe = text(-29, 5, sprintf('Safety: %s, E-Brake: %s', s_h, e_h), 'color', 'b');

	    [p_OEV, l_OEV] = plotCar(OEV.traj(1,i), OEV.traj(2,i), OEV.traj(3,i), OEV.width, OEV.length, OEV_plt_opts);
	    [p_EV, l_EV] = plotCar(z_traj(1,i), z_traj(2,i), z_traj(3,i), OEV.width, OEV.length, EV_plt_opts);
	    
	    p_TV = [];
	    l_TV = [];
	    for j = 1:N+1
	        if j == 1
	            TV_plt_opts.alpha = 0.5;
	            TV_plt_opts.frame = true;
	        else
	            TV_plt_opts.alpha = 0;
	            TV_plt_opts.frame = false;
	        end
	        
	        [p, l] = plotCar(TV.x(i+j-1), TV.y(i+j-1), TV.heading(i+j-1), TV.width, TV.length, TV_plt_opts);
	        p_TV = [p_TV, p];
	        l_TV = [l_TV, l];
	        
	        if ~isnan(hyp{j}.pos)
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
	        l_TV = [l_TV plot(z_ref(1,j), z_ref(2,j), 'o', 'color', cmap(j,:))];
	        if ~safety(i)
	            l_TV = [l_TV plot(z_pred(1,j), z_pred(2,j), 'd', 'color', cmap(j,:))];
	        end
	    end
	    axis equal
	    axis(map_dim);

	    axes(ax2)
	    axis auto
	    axis(prob_dim);
	    drawnow

	%     pause(0.05)
	    F(i) = getframe(fig);
	end

	if plt_params.mv_save
		%% Save Movie
		if ~isfolder('../../movies/')
		    mkdir('../../movies')
		end

		[file,path] = uiputfile(sprintf('../../movies/%s_Exp%d_%s.mp4', ...
		                    plt_params.mv_name, exp_params.exp_num, datestr(now,'yyyy-mm-dd_HH-MM')));

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