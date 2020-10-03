function [d, H, h] = collision_boundary_inflated(phi, w, l, r, vel_dir, scaling)
    % In this function, we represent the boundary of the collision buffer 
    % surrounding a vehicle of width w and length l with radius r in polar
    % coordinates with phi = 0 aligned with the +x direction in the 
    % vehicle body frame. d is distance to the origin of the body fixed
    % frame, H'q-h=0 is the tangent plane with outward normal at the 
    % boundary point corresponding to polar coordinates (phi, d)
    
    assert(scaling >= 1, 'Collision buffer inflation scale must be >= 1')
    
    if vel_dir > 0
        l_p = scaling*l/2;
        l_m = 1/2;
    else
        l_p = l/2;
        l_m = scaling*l/2;
    end
    
    phi_t1 = atan((w/2)/(l_p+r));
    phi_t2 = atan((w/2+r)/(l_p));
    phi_t3 = atan((w/2+r)/(l_m));
    phi_t4 = atan((w/2)/(l_m+r));
    
    % Angular position should be between 0 and 2*pi
    if phi >= 2*pi || phi < 0
        phi = mod(phi, 2*pi);
    end
    
    % Get distance of collision buffer boundary from origin at given
    % angular position
    if phi >= 0 && phi < phi_t1 || phi >= 2*pi - phi_t1 && phi < 2*pi
        d = abs((l/2+r)/cos(phi));
        H = [1; 0];
        h = l/2+r;
    elseif phi >= phi_t1 && phi < phi_t2
        p = [1+tan(phi)^2, ...
            l*tan(phi)^2-w*tan(phi), ...
            l^2*tan(phi)^2/4-w*l*tan(phi)/2+w^2/4-r^2];
        a = max(roots(p));
        b = tan(phi)*(a+l/2)-w/2;
        d = sqrt((a+l/2)^2+(b+w/2)^2);
        
        c = [l/2; w/2]; % Center of circle
        q = [d*cos(phi); d*sin(phi)]; % x-y position on boundary
        H = 2*(q-c);
        h = H'*q;
    elseif phi >= phi_t2 && phi < pi - phi_t3
        d = abs((w/2+r)/sin(phi));
        H = [0; 1];
        h = w/2+r;
    elseif phi >= pi - phi_t3 && phi < pi - phi_t4
        p = [1+tan(phi)^2, ...
            l*tan(phi)^2+w*tan(phi), ...
            l^2*tan(phi)^2/4+w*l*tan(phi)/2+w^2/4-r^2];
        a = max(roots(p));
        b = tan(phi)*(a+l/2)-w/2;
        d = sqrt((a+l/2)^2+(b+w/2)^2);
        
        c = [-l/2; w/2]; % Center of circle
        q = [d*cos(phi); d*sin(phi)]; % x-y position on boundary
        H = 2*(q-c);
        h = H'*q;
    elseif phi >= pi - phi_t4 && phi < pi + phi_t4
        d = abs((l/2+r)/cos(phi));
        H = [-1; 0];
        h = l/2+r;
    elseif phi >= pi + phi_t4 && phi < pi + phi_t3
        p = [1+tan(phi)^2, ...
            l*tan(phi)^2-w*tan(phi), ...
            l^2*tan(phi)^2/4-w*l*tan(phi)/2+w^2/4-r^2];
        a = max(roots(p));
        b = tan(phi)*(a+l/2)-w/2;
        d = sqrt((a+l/2)^2+(b+w/2)^2);
        
        c = [-l/2; -w/2]; % Center of circle
        q = [d*cos(phi); d*sin(phi)]; % x-y position on boundary
        H = 2*(q-c);
        h = H'*q;
    elseif phi >= pi + phi_t3 && phi < 2*pi - phi_t2
        d = abs((w/2+r)/sin(phi));
        H = [0; -1];
        h = w/2+r;
    elseif phi >= 2*pi - phi_t2 && phi < 2*pi - phi_t1
        p = [1+tan(phi)^2, ...
            l*tan(phi)^2+w*tan(phi), ...
            l^2*tan(phi)^2/4+w*l*tan(phi)/2+w^2/4-r^2];
        a = max(roots(p));
        b = tan(phi)*(a+l/2)-w/2;
        d = sqrt((a+l/2)^2+(b+w/2)^2);
        
        c = [l/2; -w/2]; % Center of circle
        q = [d*cos(phi); d*sin(phi)]; % x-y position on boundary
        H = 2*(q-c);
        h = H'*q;
    end
end