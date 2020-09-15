function tv_poly_obs = get_static_poly_obs(pred, wid, len)
    N = size(pred, 2);
    tv_poly_obs = cell(1, N);
    
    for k = 1:N
        center_x = pred(1,k);
        center_y = pred(2,k);
        heading = pred(3,k);

        % Target Vehicle
        Vx = [center_x + len/2*cos(heading) - wid/2*sin(heading);
              center_x + len/2*cos(heading) + wid/2*sin(heading);
              center_x - len/2*cos(heading) + wid/2*sin(heading);
              center_x - len/2*cos(heading) - wid/2*sin(heading)];

        Vy = [center_y + len/2*sin(heading) + wid/2*cos(heading);
              center_y + len/2*sin(heading) - wid/2*cos(heading);
              center_y - len/2*sin(heading) - wid/2*cos(heading);
              center_y - len/2*sin(heading) + wid/2*cos(heading)];

        P_V = Polyhedron('V', [Vx, Vy]);
        
        tv_poly_obs{1,k}.A = P_V.A;
        tv_poly_obs{1,k}.b = P_V.b;
    end
end