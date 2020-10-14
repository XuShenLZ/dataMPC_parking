%% plotCar: plot rectangle box
% plt_ops: struct with color and alpha
function [p, l] = plotCar(center_x, center_y, heading, wid, len, plt_ops)
    p = []; % Array collecting patches
    l = []; % Array collecting lines
    
	Vx = [center_x + len/2*cos(heading) - wid/2*sin(heading),
		  center_x + len/2*cos(heading) + wid/2*sin(heading),
		  center_x - len/2*cos(heading) + wid/2*sin(heading),
		  center_x - len/2*cos(heading) - wid/2*sin(heading)];

	Vy = [center_y + len/2*sin(heading) + wid/2*cos(heading),
		  center_y + len/2*sin(heading) - wid/2*cos(heading),
		  center_y - len/2*sin(heading) - wid/2*cos(heading),
		  center_y - len/2*sin(heading) + wid/2*cos(heading)];

	p = [p patch(Vx, Vy, plt_ops.color, 'FaceAlpha', plt_ops.alpha)];
    hold on
    
    if plt_ops.frame
        % Plot the body fixed frame
        R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
        body_x_dir_glob = R(heading)*[wid/3; 0] + [center_x; center_y];
        body_y_dir_glob = R(heading)*[0; wid/3] + [center_x; center_y];
        l = [l plot([center_x, body_x_dir_glob(1)], [center_y, body_x_dir_glob(2)], 'r', 'linewidth', 1.5)];
        l = [l plot([center_x, body_y_dir_glob(1)], [center_y, body_y_dir_glob(2)], 'b', 'linewidth', 1.5)];
    end
    
    % Plot the Circumcircle of vehicle
	if plt_ops.circle
		radius = 0.5 * sqrt(wid^2 + len^2);
		corner = [center_x-radius, center_y-radius];
		p = [p rectangle('Position', [corner, radius*2, radius*2], 'Curvature', [1, 1])];
	end
