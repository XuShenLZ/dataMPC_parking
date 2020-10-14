classdef StrategyPredictor < handle
	properties
		model; % Model used for prediction

		% Parameters for KF
		V = 0.01 * eye(3);
		W = 0.5 * eye(3);
		Pm = 0.2 * eye(3);

		% Initialize score
		score = ones(3, 1) / 3;
	end

	methods
		%% StrategyPredictor: constructor
		function self = StrategyPredictor(model_file, V, W, Pm)
			self.model = load(model_file);

			self.V = V;
			self.W = W;
			self.Pm = Pm;
		end

		%% predict: function description
		function score = predict(self, rel_state)
			% Predict strategy to use based on relative prediction of target
			% vehicle
			X = reshape(rel_state, [], 1);
			score_z = self.model.net(X);

			[self.score, self.Pm] = score_KF(self.score, score_z, self.V, self.W, self.Pm);
	
			score = self.score;
		end
	end

end

		
