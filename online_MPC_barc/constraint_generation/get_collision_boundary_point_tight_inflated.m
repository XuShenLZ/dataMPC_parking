function [x_b, y_b, H, h] = get_collision_boundary_point_tight_inflated(x, y, phi, w, l, r, vel_dir, scaling)
    % In this function, we represent the boundary of the collision buffer 
    % surrounding a vehicle of width w and length l with radius r in polar
    % coordinates with phi = 0 aligned with the +x direction in the 
    % offset vehicle body frame at (x, y) with respect to the frame fixed at 
    % the geometric center of the car. H'q-h=0 is the tangent plane with outward normal at the 
    % boundary point x_b, y_b
    
    % bias_ang*bias_mult is added to the equivalent angle. bias_mult between
    % 0 and 1
    
    assert(scaling >= 1, 'Collision buffer inflation scale must be >= 1')
    
    if vel_dir > 0
        l_p = scaling*l/2;
        l_m = l/2;
    else
        l_p = l/2;
        l_m = scaling*l/2;
    end
    
    if x > l_p+r || x < -(l_m+r)
        warning('Offset origin outside of collision buffer')
    end
    
    % Angular position should be between 0 and 2*pi
    if phi >= 2*pi || phi < 0
        phi = mod(phi, 2*pi);
    end
    
    % Compute equivalent origin and angle. The four options for equivalent
    % origin are the vertices of the vehicle in its body fixed frame. Each
    % vertex corresponds to a quarter of the circle. 
    if abs(y) <= w/2 
        a_1 = atan2(w/2-y, l_p+r-x); % front left
        a_2 = atan2(w/2+r-y, l_p-x); % left front
        a_3 = atan2(w/2+r-y, -l_m-x); % left rear
        a_4 = atan2(w/2-y, -l_m-r-x); % rear left
        a_5 = atan2(-w/2-y, -l_m-r-x); % rear right
        a_6 = atan2(-w/2-r-y, -l_m-x); % right rear
        a_7 = atan2(-w/2-r-y, l_p-x); % right front
        a_8 = atan2(-w/2-y, l_p+r-x); % front right
        
        % Get distance of collision buffer boundary from origin at given
        % angular position
        if phi >= 0 && phi < a_1 || phi >= 2*pi+a_8 && phi < 2*pi
            equiv_phi = 0;
        elseif phi >= a_1 && phi < a_2
            m = sin(phi)*(l_p-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = atan(b/a);
        elseif phi >= a_2 && phi < a_3
            equiv_phi = pi/2;
        elseif phi >= a_3 && phi < a_4
            m = sin(phi)*(-l_m-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi-atan(b/a);
        elseif phi >= a_4 && phi < 2*pi+a_5
            equiv_phi = pi;
        elseif phi >= 2*pi+a_5 && phi < 2*pi+a_6
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l_m-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi+atan(b/a);
        elseif phi >= 2*pi+a_6 && phi < 2*pi+a_7
            equiv_phi = 3*pi/2;
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
            m = cos(phi)*(-w/2-y)-sin(phi)*(l_p-x);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = 2*pi-atan(b/a);
        end
    elseif y > w/2 && y <= w/2+r 
        a_1 = atan2(w/2+r-y, l_p-x); % left front
        a_2 = atan2(w/2+r-y, -l_m-x); % left rear
        a_3 = atan2(w/2-y, -l_m-r-x); % rear left
        a_4 = atan2(-w/2-y, -l_m-r-x); % rear right
        a_5 = atan2(-w/2-r-y, -l_m-x); % right rear
        a_6 = atan2(-w/2-r-y, l_p-x); % right front
        a_7 = atan2(-w/2-y, l_p+r-x); % front right
        a_8 = atan2(w/2-y, l_p+r-x); % front left
        
        % Get distance of collision buffer boundary from origin at given
        % angular position
        if phi >= 0 && phi < a_1 || phi >= 2*pi+a_8 && phi < 2*pi
%             equiv_x = l/2;
%             equiv_y = w/2;
            m = sin(phi)*(l_p-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2 || phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = atan(b/a);
        elseif phi >= a_1 && phi < a_2
%             equiv_x = -l/2;
%             equiv_y = w/2;
            equiv_phi = pi/2;
        elseif phi >= a_2 && phi < 2*pi+a_3
%             equiv_x = -l/2;
%             equiv_y = w/2;
            m = sin(phi)*(-l_m-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi-atan(b/a);
        elseif phi >= 2*pi+a_3 && phi < 2*pi+a_4
%             equiv_x = -l/2;
%             equiv_y = -w/2;
            equiv_phi = pi;
        elseif phi >= 2*pi+a_4 && phi < 2*pi+a_5
%             equiv_x = -l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l_m-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi+atan(b/a);
        elseif phi >= 2*pi+a_5 && phi < 2*pi+a_6
%             equiv_x = l/2;
%             equiv_y = -w/2;
            equiv_phi = 3*pi/2;
        elseif phi >= 2*pi+a_6 && phi < 2*pi+a_7
%             equiv_x = l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(l_p-x);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = 2*pi-atan(b/a);
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
%             equiv_x = l/2;
%             equiv_y = w/2;
            equiv_phi = 0;
        end
    elseif y < -w/2 && y >= -w/2-r
        a_1 = atan2(-w/2-y, l_p+r-x); % front right
        a_2 = atan2(w/2-y, l_p+r-x); % front left
        a_3 = atan2(w/2+r-y, l_p-x); % left front
        a_4 = atan2(w/2+r-y, -l_m-x); % left rear
        a_5 = atan2(w/2-y, -l_m-r-x); % rear left
        a_6 = atan2(-w/2-y, -l_m-r-x); % rear right
        a_7 = atan2(-w/2-r-y, -l_m-x); % right rear
        a_8 = atan2(-w/2-r-y, l_p-x); % right front
        
        % Get distance of collision buffer boundary from origin at given
        % angular position
        if phi >= 0 && phi < a_1 || phi >= 2*pi+a_8 && phi < 2*pi
%             equiv_x = l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(l_p-x);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= 3*pi/2 || phi <= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = 2*pi-atan(b/a);
        elseif phi >= a_1 && phi < a_2
%             equiv_x = l/2;
%             equiv_y = w/2;
            equiv_phi = 0;
        elseif phi >= a_2 && phi < a_3
%             equiv_x = l/2;
%             equiv_y = w/2;
            m = sin(phi)*(l_p-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2 || phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = atan(b/a);
        elseif phi >= a_3 && phi < a_4
%             equiv_x = -l/2;
%             equiv_y = w/2;
            equiv_phi = pi/2;
        elseif phi >= a_4 && phi < a_5
%             equiv_x = -l/2;
%             equiv_y = w/2;
            m = sin(phi)*(-l_m-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi-atan(b/a);
        elseif phi >= a_5 && phi < a_6
%             equiv_x = -l/2;
%             equiv_y = -w/2;
            equiv_phi = pi;
        elseif phi >= a_6 && phi < 2*pi+a_7
%             equiv_x = -l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l_m-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi+atan(b/a);
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
%             equiv_x = l/2;
%             equiv_y = -w/2;
            equiv_phi = 3*pi/2;
        end
    else
        warning('Offset origin outside of collision buffer')
    end
    
    phi = equiv_phi;
    % Angular position should be between 0 and 2*pi
    if phi >= 2*pi || phi < 0
        phi = mod(phi, 2*pi);
    end
    if phi < pi/2 && phi >= 0
        x = l_p;
        y = w/2;
    elseif phi < pi && phi >= pi/2
        x = -l_m;
        y = w/2;
    elseif phi < 3*pi/2 && phi >= pi
        x = -l_m;
        y = -w/2;
    else
        x = l_p;
        y = -w/2;
    end
    
    x_b = x + r*cos(phi);
    y_b = y + r*sin(phi);
    
    c = [x; y];
    q = [x_b; y_b];
    H = 2*(q-c);
%     h = H'*q;
    h = H'*c;
end