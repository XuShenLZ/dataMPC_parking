%% plotCar: plot rectangle box
% plt_ops: struct with color and alpha
function [p,c] = plotCar(center_x, center_y, heading, wid, len, plt_ops)
	Vx = [center_x + len/2*cos(heading) - wid/2*sin(heading),
		  center_x + len/2*cos(heading) + wid/2*sin(heading),
		  center_x - len/2*cos(heading) + wid/2*sin(heading),
		  center_x - len/2*cos(heading) - wid/2*sin(heading)];

	Vy = [center_y + len/2*sin(heading) + wid/2*cos(heading),
		  center_y + len/2*sin(heading) - wid/2*cos(heading),
		  center_y - len/2*sin(heading) - wid/2*cos(heading),
		  center_y - len/2*sin(heading) + wid/2*cos(heading)];

	p = patch(Vx, Vy, plt_ops.color, 'FaceAlpha', plt_ops.alpha);

	% Plot the Circumcircle of vehicle
	if plt_ops.circle
		radius = 0.5 * sqrt(wid^2 + len^2);
		corner = [center_x-radius, center_y-radius];
		c = rectangle('Position', [corner, radius*2, radius*2], 'Curvature', [1, 1]);
	else
		c = 0;
	end
