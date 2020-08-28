function v_min = get_closest_vertex(p, q, theta, w, l)
    x = q(1);
    y = q(2);
    car_body_x = [x + l/2*cos(theta) - w/2*sin(theta); % front left
		  x + l/2*cos(theta) + w/2*sin(theta); % front right
		  x - l/2*cos(theta) + w/2*sin(theta); % back right
		  x - l/2*cos(theta) - w/2*sin(theta)]; % back left

	car_body_y = [y + l/2*sin(theta) + w/2*cos(theta); % front left
		  y + l/2*sin(theta) - w/2*cos(theta); % front right
		  y - l/2*sin(theta) - w/2*cos(theta); % back right
		  y - l/2*sin(theta) + w/2*cos(theta)]; % back left
      
    v_car = [car_body_x, car_body_y]';
    
    [~, min_idx] = min(vecnorm(v_car - p, 2));
    
    v_min = v_car(:,min_idx);
end