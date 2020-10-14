function collision = check_collision_point_inflated(p, x, y, theta, w, l, r, vel_dir, scaling)
    % p: query point
    % (x, y, theta): configuration of target vehicle
    % (w, l, r): width, length of vehicle and collision buffer radius
    collision = false;
    
    if vel_dir > 0
        l_p = scaling*l/2;
        l_m = 1/2;
    else
        l_p = l/2;
        l_m = scaling*l/2;
    end
    
    % Get polytope of car body
    car_body_x = [x + l_p*cos(theta) - w/2*sin(theta); % front left
		  x + l_p*cos(theta) + w/2*sin(theta); % front right
		  x - l_m*cos(theta) + w/2*sin(theta); % back right
		  x - l_m*cos(theta) - w/2*sin(theta)]; % back left

	car_body_y = [y + l_p*sin(theta) + w/2*cos(theta); % front left
		  y + l_p*sin(theta) - w/2*cos(theta); % front right
		  y - l_m*sin(theta) - w/2*cos(theta); % back right
		  y - l_m*sin(theta) + w/2*cos(theta)]; % back left
    
    reg_body = Polyhedron('V', [car_body_x car_body_y]);
    if all(reg_body.A*p - reg_body.b <= 0)
        collision = true;
        return
    end
    
    % Get polytope of front collision buffer
    car_front_x = [car_body_x(1); car_body_x(2); 
            x + (l_p+r)*cos(theta) - w/2*sin(theta);
            x + (l_p+r)*cos(theta) + w/2*sin(theta)];
    car_front_y = [car_body_y(1); car_body_y(2);
            y + (l_p+r)*sin(theta) + w/2*cos(theta);
            y + (l_p+r)*sin(theta) - w/2*cos(theta)];
    
    reg_front = Polyhedron('V', [car_front_x car_front_y]);
    if all(reg_front.A*p - reg_front.b <= 0)
        collision = true;
        return
    end
    
    % Get polytope of rear collision buffer
    car_rear_x = [car_body_x(3); car_body_x(4); 
            x - (l_m+r)*cos(theta) + w/2*sin(theta);
            x - (l_m+r)*cos(theta) - w/2*sin(theta)];
    car_rear_y = [car_body_y(3); car_body_y(4);
            y - (l_m+r)*sin(theta) - w/2*cos(theta);
            y - (l_m+r)*sin(theta) + w/2*cos(theta)];
    
    reg_rear = Polyhedron('V', [car_rear_x car_rear_y]);
    if all(reg_rear.A*p - reg_rear.b <= 0)
        collision = true;
        return
    end
    
    % Get polytope of left collision buffer
    car_left_x = [car_body_x(1); car_body_x(4); 
            x + l_p*cos(theta) - (w/2+r)*sin(theta);
            x - l_m*cos(theta) - (w/2+r)*sin(theta)];
    car_left_y = [car_body_y(1); car_body_y(4);
            y + l_p*sin(theta) + (w/2+r)*cos(theta);
            y - l_m*sin(theta) + (w/2+r)*cos(theta)];
    
    reg_left = Polyhedron('V', [car_left_x car_left_y]);
    if all(reg_left.A*p - reg_left.b <= 0)
        collision = true;
        return
    end
    
    % Get polytope of right collision buffer
    car_right_x = [car_body_x(2); car_body_x(3); 
            x + l_p*cos(theta) + (w/2+r)*sin(theta);
            x - l_m*cos(theta) + (w/2+r)*sin(theta)];
    car_right_y = [car_body_y(2); car_body_y(3);
            y + l_p*sin(theta) - (w/2+r)*cos(theta);
            y - l_m*sin(theta) - (w/2+r)*cos(theta)];
    
    reg_right = Polyhedron('V', [car_right_x car_right_y]);
    if all(reg_right.A*p - reg_right.b <= 0)
        collision = true;
        return
    end
    
    % Get circular region of front left collision buffer
    front_left_coord = [car_body_x(1); car_body_y(1)];
    if norm(p - front_left_coord, 2) - r <= 0
        collision = true;
        return
    end
    
    % Get circular region of front right collision buffer
    front_right_coord = [car_body_x(2); car_body_y(2)];
    if norm(p - front_right_coord, 2) - r <= 0
        collision = true;
        return
    end
    
    % Get circular region of rear left collision buffer
    rear_left_coord = [car_body_x(4); car_body_y(4)];
    if norm(p - rear_left_coord, 2) - r <= 0
        collision = true;
        return
    end
    
    % Get circular region of rear right collision buffer
    rear_right_coord = [car_body_x(3); car_body_y(3)];
    if norm(p - rear_right_coord, 2) - r <= 0
        collision = true;
        return
    end
end