%% score_KF: Kalman Filter for score smoothing
function [xm, Pm] = score_KF(x, z, V, W, Pm)
	A = eye(3);
	H = eye(3);

	xp = A * x;
	Pp = Pm + V;

	xm = xp + Pp * H' / (H*Pp*H' + W) * (z - H*xp);
	Pm = Pp - Pp * H' / (H*Pp*H' + W) * H * Pp;

	xm = xm / sum(xm);