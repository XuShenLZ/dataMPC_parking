function [x_b, y_b, H, h] = get_collision_boundary_point(x, y, phi, w, l, r)
    % In this function, we represent the boundary of the collision buffer 
    % surrounding a vehicle of width w and length l with radius r in polar
    % coordinates with phi = 0 aligned with the +x direction in the 
    % offset vehicle body frame at (x, y) with respect to the frame fixed at 
    % the geometric center of the car. H'q-h=0 is the tangent plane with outward normal at the 
    % boundary point x_b, y_b
    
    if abs(x) > l/2+r
        error('Offset origin outside of collision buffer')
    end
    
    % Angular position should be between 0 and 2*pi
    if phi >= 2*pi || phi < 0
        phi = mod(phi, 2*pi);
    end
    
    % Compute angular positions where boundary changes from straight to
    % curved section
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
            d = abs((l/2+r-x)/cos(phi));
            H = [1; 0];
            h = l/2+r;
        elseif phi >= a_1 && phi < a_2
            m = sin(phi)*(l/2-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((a+l/2-x)^2+(b+w/2-y)^2);

            c = [l/2; w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= a_2 && phi < a_3
            d = abs((w/2+r-y)/sin(phi));
            H = [0; 1];
            h = w/2+r;
        elseif phi >= a_3 && phi < a_4
            m = sin(phi)*(-l/2-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((-a-l/2-x)^2+(b+w/2-y)^2);

            c = [-l/2; w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= a_4 && phi < 2*pi+a_5
            d = abs((-l/2-r-x)/cos(phi));
            H = [-1; 0];
            h = l/2+r;
        elseif phi >= 2*pi+a_5 && phi < 2*pi+a_6
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l/2-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((-a-l/2-x)^2+(-b-w/2-y)^2);

            c = [-l/2; -w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= 2*pi+a_6 && phi < 2*pi+a_7
            d = abs((-w/2-r-y)/sin(phi));
            H = [0; -1];
            h = w/2+r;
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
            m = cos(phi)*(-w/2-y)-sin(phi)*(l/2-x);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((a+l/2-x)^2+(-b-w/2-y)^2);

            c = [l/2; -w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
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
            m = sin(phi)*(l/2-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2 || phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((a+l/2-x)^2+(b+w/2-y)^2);

            c = [l/2; w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= a_1 && phi < a_2
            d = abs((w/2+r-y)/sin(phi));
            H = [0; 1];
            h = w/2+r;
        elseif phi >= a_2 && phi < 2*pi+a_3
            m = sin(phi)*(-l/2-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((-a-l/2-x)^2+(b+w/2-y)^2);

            c = [-l/2; w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= 2*pi+a_3 && phi < 2*pi+a_4
            d = abs((-l/2-r-x)/cos(phi));
            H = [-1; 0];
            h = l/2+r;
        elseif phi >= 2*pi+a_4 && phi < 2*pi+a_5
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l/2-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((-a-l/2-x)^2+(-b-w/2-y)^2);

            c = [-l/2; -w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= 2*pi+a_5 && phi < 2*pi+a_6
            d = abs((-w/2-r-y)/sin(phi));
            H = [0; -1];
            h = w/2+r;
        elseif phi >= 2*pi+a_6 && phi < 2*pi+a_7
            m = cos(phi)*(-w/2-y)-sin(phi)*(l/2-x);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((a+l/2-x)^2+(-b-w/2-y)^2);

            c = [l/2; -w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
            d = abs((l/2+r-x)/cos(phi));
            H = [1; 0];
            h = l/2+r;
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
            m = cos(phi)*(-w/2-y)-sin(phi)*(l/2-x);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= 3*pi/2 || phi <= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((a+l/2-x)^2+(-b-w/2-y)^2);

            c = [l/2; -w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= a_1 && phi < a_2
            d = abs((l/2+r-x)/cos(phi));
            H = [1; 0];
            h = l/2+r;
        elseif phi >= a_2 && phi < a_3
            m = sin(phi)*(l/2-x)-cos(phi)*(w/2-y);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= pi/2 || phi >= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((a+l/2-x)^2+(b+w/2-y)^2);

            c = [l/2; w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= a_3 && phi < a_4
            d = abs((w/2+r-y)/sin(phi));
            H = [0; 1];
            h = w/2+r;
        elseif phi >= a_4 && phi < a_5
            m = sin(phi)*(-l/2-x)-cos(phi)*(w/2-y);
            p = [1, -2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi >= pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((-a-l/2-x)^2+(b+w/2-y)^2);

            c = [-l/2; w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= a_5 && phi < a_6
            d = abs((-l/2-r-x)/cos(phi));
            H = [-1; 0];
            h = l/2+r;
        elseif phi >= a_6 && phi < 2*pi+a_7
            m = cos(phi)*(-w/2-y)-sin(phi)*(-l/2-x);
            p = [1, 2*sin(phi)*m, m^2-cos(phi)^2*r^2];
            if phi <= 3*pi/2
                a = max(roots(p));
            else
                a = min(roots(p));
            end
            b = sqrt(r^2-a^2);
            d = sqrt((-a-l/2-x)^2+(-b-w/2-y)^2);

            c = [-l/2; -w/2]; % Center of circle
            q = [d*cos(phi)+x; d*sin(phi)+y]; % x-y position on boundary
            H = 2*(q-c);
            h = H'*q;
        elseif phi >= 2*pi+a_7 && phi < 2*pi+a_8
            d = abs((-w/2-r-y)/sin(phi));
            H = [0; -1];
            h = w/2+r;
        end
    else
        error('Offset origin outside of collision buffer')
    end
    
    x_b = d*cos(phi) + x;
    y_b = d*sin(phi) + y;
end