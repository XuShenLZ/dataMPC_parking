%% gen_feature_label: generate feature and label from ground truth training_data
% generated along with hyperplanes
function [feature, label, ego_ref] = gen_feature_label(training_data, t, N)
	% The position of ego at current time
	ego_ref = training_data(t).EV_N.state(1:2, 1);

	% [x_TV-x_EV; y_TV-y_EV; heading_EV; heading_TV]
	feature = zeros(4, N+1);
	for k = 0:N
		% Local x y
		feature(1:2, k+1) = training_data(t+k).TV_N.state(1:2, 1) - training_data(t+k).EV_N.state(1:2, 1);
		% Global heading
		feature(3,   k+1) = training_data(t+k).EV_N.state(3, 1);
		feature(4,   k+1) = training_data(t+k).TV_N.state(3, 1);
	end

	% [w1; w2; b]
	label = zeros(3, N+1);
	for k = 0:N
		label(1:2, k+1) = training_data(t+k).hyperplane.w;
		% Need to transform b into local ego frame
		label(3,   k+1) = training_data(t+k).hyperplane.b - label(1:2, k+1)'*ego_ref;
	end
