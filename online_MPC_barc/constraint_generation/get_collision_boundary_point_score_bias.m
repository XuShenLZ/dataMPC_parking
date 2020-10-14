function [x_b, y_b, H, h] = get_collision_boundary_point_score_bias(x, y, phi, w, l, r, bias_ang, bias_mult)
    % In this function, we represent the boundary of the collision buffer 
    % surrounding a vehicle of width w and length l with radius r in polar
    % coordinates with phi = 0 aligned with the +x direction in the 
    % offset vehicle body frame at (x, y) with respect to the frame fixed at 
    % the geometric center of the car. H'q-h=0 is the tangent plane with outward normal at the 
    % boundary point x_b, y_b
    
    % bias_ang*bias_mult is added to the equivalent angle. bias_mult between
    % 0 and 1
    
    if abs(x) > l/2+r
        error('Offset origin outside of collision buffer')
    end
    
    % Angular position should be between 0 and 2*pi
    if phi >= 2*pi || phi < 0
        phi = mod(phi, 2*pi);
    end
    
    % Compute equivalent origin and angle. The four options for equivalent
    % origin are the vertices of the vehicle in its body fixed frame. Each
    % vertex corresponds to a quarter of the circle. 
    if abs(y) <= w/2 
        a_1 = atan2(w/2-y, l/2+r-x); % front left
        a_2 = atan2(w/2+r-y, l/2-x); % left front
        a_3 = atan2(w/2+r-y, -l/2-x); % left rear
        a_4 = atan2(w/2-y, -l/2-r-x); % rear left
        a_5 = atan2(-w/2-y, -l/2-r-x); % rear right
        a_6 = atan2(-w/2-r-y, -l/2-x); % right rear
        a_7 = atan2(-w/2-r-y, l/2-x); % right front
        a_8 = atan2(-w/2-y, l/2+r-x); % front right
        
        % Get distance of collision buffer boundary from origin at given
        % angular position
        if phi >= 0 && phi < a_1 || phi >= 2*pi+a_8 && phi < 2*pi
%             equiv_x = l/2;
%             equiv_y = w/2;
            equiv_phi = 0;
        elseif phi >= a_1 && phi < a_2
%             equiv_x = l/2;
%             equiv_y = w/2;
            m = sin(phi)*(l/2-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = atan(b/a);
        elseif phi >= a_2 && phi < a_3
%             equiv_x = -l/2;
%             equiv_y = w/2;
            equiv_phi = pi/2;
        elseif phi >= a_3 && phi < a_4
%             equiv_x = -l/2;
%             equiv_y = w/2;
            m = sin(phi)*(-l/2-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi-atan(b/a);
        elseif phi >= a_4 && phi < 2*pi+a_5
%             equiv_x = -l/2;
%             equiv_y = -w/2;
            equiv_phi = pi;
        elseif phi >= 2*pi+a_5 && phi < 2*pi+a_6
%             equiv_x = -l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l/2-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            equiv_phi = pi+atan(b/a);
        elseif phi >= 2*pi+a_6 && phi < 2*pi+a_7
%             equiv_x = l/2;
%             equiv_y = -w/2;
            equiv_phi = 3*pi/2;
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
%             equiv_x = l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(l/2-x);
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
        a_1 = atan2(w/2+r-y, l/2-x); % left front
        a_2 = atan2(w/2+r-y, -l/2-x); % left rear
        a_3 = atan2(w/2-y, -l/2-r-x); % rear left
        a_4 = atan2(-w/2-y, -l/2-r-x); % rear right
        a_5 = atan2(-w/2-r-y, -l/2-x); % right rear
        a_6 = atan2(-w/2-r-y, l/2-x); % right front
        a_7 = atan2(-w/2-y, l/2+r-x); % front right
        a_8 = atan2(w/2-y, l/2+r-x); % front left
        
        % Get distance of collision buffer boundary from origin at given
        % angular position
        if phi >= 0 && phi < a_1 || phi >= 2*pi+a_8 && phi < 2*pi
%             equiv_x = l/2;
%             equiv_y = w/2;
            m = sin(phi)*(l/2-x)-cos(phi)*(w/2-y);
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
            m = sin(phi)*(-l/2-x)-cos(phi)*(w/2-y);
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
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l/2-x);
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
            m = cos(phi)*(-w/2-y)-sin(phi)*(l/2-x);
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
        a_1 = atan2(-w/2-y, l/2+r-x); % front right
        a_2 = atan2(w/2-y, l/2+r-x); % front left
        a_3 = atan2(w/2+r-y, l/2-x); % left front
        a_4 = atan2(w/2+r-y, -l/2-x); % left rear
        a_5 = atan2(w/2-y, -l/2-r-x); % rear left
        a_6 = atan2(-w/2-y, -l/2-r-x); % rear right
        a_7 = atan2(-w/2-r-y, -l/2-x); % right rear
        a_8 = atan2(-w/2-r-y, l/2-x); % right front
        
        % Get distance of collision buffer boundary from origin at given
        % angular position
        if phi >= 0 && phi < a_1 || phi >= 2*pi+a_8 && phi < 2*pi
%             equiv_x = l/2;
%             equiv_y = -w/2;
            m = cos(phi)*(-w/2-y)-sin(phi)*(l/2-x);
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
            m = sin(phi)*(l/2-x)-cos(phi)*(w/2-y);
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
            m = sin(phi)*(-l/2-x)-cos(phi)*(w/2-y);
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
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l/2-x);
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
        error('Offset origin outside of collision buffer')
    end
    
    phi_diff = bias_ang - equiv_phi;
    if phi_diff > pi
        phi_diff = phi_diff - 2*pi;
    elseif phi_diff < -pi
        phi_diff = phi_diff + 2*pi;
    end
    
    phi = equiv_phi + phi_diff*bias_mult;
    % Angular position should be between 0 and 2*pi
    if phi >= 2*pi || phi < 0
        phi = mod(phi, 2*pi);
    end
    if phi < pi/2 && phi >= 0
        x = l/2;
        y = w/2;
    elseif phi < pi && phi >= pi/2
        x = -l/2;
        y = w/2;
    elseif phi < 3*pi/2 && phi >= pi
        x = -l/2;
        y = -w/2;
    else
        x = l/2;
        y = -w/2;
    end
    
    x_b = x + r*cos(phi);
    y_b = y + r*sin(phi);
    
    c = [x; y];
    q = [x_b; y_b];
    H = 2*(q-c);
    h = H'*q;
end