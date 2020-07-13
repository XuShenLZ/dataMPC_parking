%% hybrid_A_star: function description
function [refPath, planner] = hybrid_A_star(img, startPose, goalPose, ops)

	% Vehicle Config
	vehicleDims = vehicleDimensions(ops.length, ops.width, ...
									'FrontOverhang', 0.2*ops.length,...
									'RearOverhang', 0.2*ops.length);

	distFromSide = 0.5 * ops.width / ops.length;
	centerPlacements = [distFromSide 0.5 1-distFromSide];

	map = vehicleCostmap(double(img));
	map.CollisionChecker = inflationCollisionChecker(vehicleDims, ...
													 'CenterPlacements', centerPlacements,...
													 'InflationRadius', ops.inflate_radius);

	% ====== Visualize the collision checker
	% figure
	% plot(map.CollisionChecker)

	% Hybrid A* Config
	validator = validatorVehicleCostmap;

	validator.Map = map;

	planner = plannerHybridAStar(validator, ...
								'InterpolationDistance', ops.InterpolationDistance, ...
								'MinTurningRadius', ops.MinTurningRadius, ...
								'MotionPrimitiveLength', ops.MotionPrimitiveLength, ...
								'ReverseCost', ops.ReverseCost);
	try
		path_struct = plan(planner, startPose', goalPose');
		refPath = path_struct.States;
	catch
		warning('Hybrid A* is not feasible. Return an empty refPath.');
		refPath = [];
% 		keyboard
	end
	% refPath = refPath';

	% ====== Visualize
	% fig_astar = figure;
	% plot(map)
	% title('Cost Map for Hybrid A*')
	% hold on

	% plot(startPose(1), startPose(2), 'o', 'markersize', 10, 'DisplayName', 'Start')
	% hold on
	% plot(goalPose(1), goalPose(2), 'x', 'markersize', 10, 'DisplayName', 'Goal')
	% hold on

	% show(planner)
	% close(fig_astar)