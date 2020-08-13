function [hyp_xy, hyp_w, hyp_b] = get_extreme_pt_hyp_tight(p, dir, x, y, theta, w, l, r)  
    R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    % Transform point in global frame to local body fixed frame
    p_loc = R(theta)\(p-[x; y]);
    dir_loc = R(theta)\(dir);
    phi_loc = atan2(dir_loc(2), dir_loc(1));
    
    [x_local, y_local, hyp_w_local, hyp_b_local] = get_collision_boundary_point_tight(p_loc(1), p_loc(2), phi_loc, ...
        w, l, r);
    
    % Point where hyperplane is tangent to collision boundary
    hyp_xy = R(theta)*[x_local; y_local] + [x; y];
    
    % Transform hyperplane into global frame
    hyp_w = R(theta)*hyp_w_local;
    hyp_b = hyp_w'*[x; y] + hyp_b_local;

    % Debug plotting
%     phi = linspace(0, 2*pi, 200);
%     x = [];
%     y = [];
%     for i = 1:length(phi)
%         [d, ~, ~] = collision_boundary(phi(i), w, l, r);
%         x = [x d*cos(phi(i))];
%         y = [y d*sin(phi(i))];
%     end
%     
%     TV_plt_opts.circle = false;
%     TV_plt_opts.color = 'r';
%     TV_plt_opts.alpha = 0.5;
% 
%     figure()
%     plot(x, y)
%     hold on
%     plotCar(0, 0, 0, w, l, TV_plt_opts);
%     plot(p_loc(1), p_loc(2), 'go')
%     plot([p_loc(1) p_loc(1)+dir_loc(1)], [p_loc(2) p_loc(2)+dir_loc(2)], 'g')
%     plot(x_local, y_local, 'go')
%     plot([p_loc(1) p_loc(1)+1], [p_loc(2) p_loc(2)], 'r--', 'linewidth', 1.5)
%     plot([p_loc(1) p_loc(1)], [p_loc(2) p_loc(2)+1], 'b--', 'linewidth', 1.5)
%     axis equal
end
    