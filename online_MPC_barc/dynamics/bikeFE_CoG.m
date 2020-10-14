%% bikeFE_CoG: kinematic bicycle model in center of gravity
% z: [x, y, psi, v]'
% u: [beta, a]
% beta = atan(lr/(lf+lr)*tan(delta));
function zp = bikeFE_CoG(z, u, L, dt)

	% ======== CoG Reference
	lf = L/2;
	lr = L/2;

	zp = z;

	% psi_new = z(3) + dt/2 * z(4) / lr * sin(u(1));
	v_new = z(4) + dt/2 * u(2);

	zp(1) = z(1) + dt * v_new * cos(z(3) + u(1));
	zp(2) = z(2) + dt * v_new * sin(z(3) + u(1));
	zp(3) = z(3) + dt * v_new / lr * sin(u(1));
	zp(4) = z(4) + dt * u(2);

	% ======== Rear Wheel Axis Reference
	% zp = z;

	% psi_new = z(3) + dt/2 * z(4) / L * tan(u(1));
	% v_new = z(4) + dt/2 * u(2);

	% zp(1) = z(1) + dt * v_new * cos(psi_new);
	% zp(2) = z(2) + dt * v_new * sin(psi_new);
	% zp(3) = z(3) + dt * v_new / L * tan(u(1));
	% zp(4) = z(4) + dt * u(2);

