%% entend_prevItr: uses the previous z_opt and extend one time step
% for the ref/warm start at the next time step.
% Extend under the constant input assumption
function [z_WS, u_WS] = entend_prevItr(z_opt, u_opt, EV)
	dt = EV.dt;
	offset = EV.offset;
	L = EV.L;

	z_WS = z_opt(:, 2:end);
	u_WS = u_opt(:, 2:end);

	u_last = u_WS(:, end);
	z_last = z_WS(:, end);

	z_last(1) = z_last(1) - offset*cos(z_last(3));
	z_last(2) = z_last(2) - offset*sin(z_last(3));

	z_plus = bikeFE(z_last, u_last, L, dt);

	z_plus(1) = z_plus(1) + offset*cos(z_plus(3));
	z_plus(2) = z_plus(2) + offset*sin(z_plus(3));

	z_WS = [z_WS, z_plus];
	u_WS = [u_WS, u_last];