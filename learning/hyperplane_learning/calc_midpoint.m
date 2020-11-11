%% calc_midpoint: calculate midpoint by giving the 
% x,y,theta of EV and TV
function [m] = calc_midpoint(EV_xyt, TV_xyt, EV, TV)
	EV_Vx_curr = [EV_xyt(1) + EV.length/2*cos(EV_xyt(3)) - EV.width/2*sin(EV_xyt(3));
		  EV_xyt(1) + EV.length/2*cos(EV_xyt(3)) + EV.width/2*sin(EV_xyt(3));
		  EV_xyt(1) - EV.length/2*cos(EV_xyt(3)) + EV.width/2*sin(EV_xyt(3));
		  EV_xyt(1) - EV.length/2*cos(EV_xyt(3)) - EV.width/2*sin(EV_xyt(3))];
    EV_Vy_curr = [EV_xyt(2) + EV.length/2*sin(EV_xyt(3)) + EV.width/2*cos(EV_xyt(3));
		  EV_xyt(2) + EV.length/2*sin(EV_xyt(3)) - EV.width/2*cos(EV_xyt(3));
		  EV_xyt(2) - EV.length/2*sin(EV_xyt(3)) - EV.width/2*cos(EV_xyt(3));
		  EV_xyt(2) - EV.length/2*sin(EV_xyt(3)) + EV.width/2*cos(EV_xyt(3))];
	EV_V_curr = [EV_Vx_curr, EV_Vy_curr];

	TV_Vx_curr = [TV_xyt(1) + TV.length/2*cos(TV_xyt(3)) - TV.width/2*sin(TV_xyt(3));
		  TV_xyt(1) + TV.length/2*cos(TV_xyt(3)) + TV.width/2*sin(TV_xyt(3));
		  TV_xyt(1) - TV.length/2*cos(TV_xyt(3)) + TV.width/2*sin(TV_xyt(3));
		  TV_xyt(1) - TV.length/2*cos(TV_xyt(3)) - TV.width/2*sin(TV_xyt(3))];
    TV_Vy_curr = [TV_xyt(2) + TV.length/2*sin(TV_xyt(3)) + TV.width/2*cos(TV_xyt(3));
		  TV_xyt(2) + TV.length/2*sin(TV_xyt(3)) - TV.width/2*cos(TV_xyt(3));
		  TV_xyt(2) - TV.length/2*sin(TV_xyt(3)) - TV.width/2*cos(TV_xyt(3));
		  TV_xyt(2) - TV.length/2*sin(TV_xyt(3)) + TV.width/2*cos(TV_xyt(3))];
	TV_V_curr = [TV_Vx_curr, TV_Vy_curr];

	% Compute midpoint between current ego and target vehicle occupied
    % areas
    EV_P = Polyhedron('V', EV_V_curr);
    TV_P = Polyhedron('V', TV_V_curr);
    s = EV_P.distance(TV_P);
    m = (s.y + s.x)/2;