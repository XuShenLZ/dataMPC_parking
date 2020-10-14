%% extend_prevItr: uses the previous z_opt and extend one time step
% for the ref/warm start at the next time step.
% Extend under the constant input assumption
function [z_WS, u_WS] = extend_prevItr(z_opt, u_opt, dt, L)

	z_WS = z_opt(:, 2:end);
	u_WS = u_opt(:, 2:end);

	u_last = u_WS(:, end);
	z_last = z_WS(:, end);

	z_plus = bikeFE_CoG(z_last, u_last, L, dt);

	z_WS = [z_WS, z_plus];
	u_WS = [u_WS, u_last];