%% plotExp: plot the experiment process
function F = plotTracking(dataname, plt_params)
    close all;
    
	load(dataname)
    
    if ~isfield(exp_params, 'map_dim')
        map_dim = [-30 30 -10 10];
    else
        map_dim = exp_params.map_dim;
    end
    
	T = exp_params.T;
	N = exp_params.controller.N;
    
    % Default plot parameter values
    if isfield(plt_params, 'plt_ref')
        plt_ref = plt_params.plt_ref;
    else
        plt_ref = false;
    end
    if isfield(plt_params, 'plt_preds')
        plt_preds = plt_params.plt_preds;
    else
        plt_preds = true;
    end
    plt_tv_preds = plt_params.plt_tv_preds;

	EV_plt_opts.frame = true;
	EV_plt_opts.color = 'b';
	EV_plt_opts.alpha = 0.5;
    EV_plt_opts.circle = false;

	TV_plt_opts.color = 'y';

	cmap = jet(N+1);
	p_EV = []; l_EV = [];
	p_TV = []; l_TV = [];

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

	% Plot inputs
	ax_h_v = axes('Position',[0.5 0.75 0.45 0.2]);
	h_v_l = animatedline(ax_h_v, 'color', '#0072BD', 'linewidth', 2);
	ylabel('v')
	xlim([1 T-N])

	ax_h_d = axes('Position',[0.5 0.52 0.45 0.2]);
	h_d_l = animatedline(ax_h_d, 'color', '#0072BD', 'linewidth', 2);
	ylabel('delta')
	xlim([1 T-N])

	ax_h_a = axes('Position',[0.5 0.28 0.45 0.2]);
	h_a_l = animatedline(ax_h_a, 'color', '#0072BD', 'linewidth', 2);
	ylabel('a')
	xlim([1 T-N])

    for i = 1:T-N
        % Delete lines and patches from last iteration
	    delete(p_EV); delete(l_EV); 
	    delete(p_TV); delete(l_TV); 
        
	    z_ref = z_refs(:,:,i);
	    z_pred = z_preds(:,:,i);
	    u_pred = u_preds(:,:,i);
	    
        if z_traj(1,i) > map_dim(2)
	        F(i:end) = [];
	        break;
        end
	    
	    addpoints(h_v_l, i, z_traj(4,i));
	    addpoints(h_d_l, i, u_traj(1,i));
	    addpoints(h_a_l, i, u_traj(2,i));
        
        axes(ax1)
        [p_EV, l_EV] = plotCar(z_traj(1,i), z_traj(2,i), z_traj(3,i), TV.width, TV.length, EV_plt_opts);
        
	    p_TV = [];
	    l_TV = [];
        
        % Plot TV over horizon and predictions
        for j = 1:N+1
            if j == 1
	            TV_plt_opts.alpha = 0.5;
	            TV_plt_opts.frame = true;
                TV_plt_opts.circle = false;
                
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
            
            if plt_ref
                l_TV = [l_TV plot(z_ref(1,j), z_ref(2,j), '.', 'color', cmap(j,:), 'markersize', 10)];
            end
            
            if plt_preds
                l_TV = [l_TV plot(z_pred(1,j), z_pred(2,j), 'd', 'color', cmap(j,:))];
            end
        end
        
	    axis equal
	    axis(map_dim);
        
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