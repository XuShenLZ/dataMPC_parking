%% bikeFE: kinematic bicycle model
% z: [x, y, psi, v]'
% u: [delta, a]
function zp = bikeFE(z, u, L, dt)

	% beta = atan(lr/(lf+lr)*tan(u(1)));

	% zp = z;

	% zp(1) = z(1) + dt * z(3) * cos(z(4) + beta);
	% zp(2) = z(2) + dt * z(3) * sin(z(4) + beta);
	% zp(3) = z(3) + dt * u(2);
	% zp(4) = z(4) + dt * z(3) / lr * sin(beta);

	zp = z;

	psi_new = z(3) + dt/2 * z(4) / L * tan(u(1));
	v_new = z(4) + dt/2 * u(2);

	zp(1) = z(1) + dt * v_new * cos(psi_new);
	zp(2) = z(2) + dt * v_new * sin(psi_new);
	zp(3) = z(3) + dt * v_new / L * tan(u(1));
	zp(4) = z(4) + dt * u(2);

