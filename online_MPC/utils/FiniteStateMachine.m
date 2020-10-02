classdef FiniteStateMachine < handle
	properties
		state = "Start"; % current state
		strategy = "N/A"; % Current strategy

		N; % Horizon Length
		r; % Collision buffer radius
		confidence_thresh; % Confidence Threshold for Strategy
		x_max; % Maximum x coordinate to finish the task
		T_max; % Maximum time steps for the task
		T_tv; % Length of tv data
		lock_steps; % Steps to lock the HOBCA
		strategy_names; % Names of all strategies

		a_lim; % Acceleration Limit
		dt; % time interval
	end

	methods
		%% FSM: constructor 
		function self = FiniteStateMachine(exp_params)
			self.N = exp_params.controller.N;
			self.r = exp_params.r;
			self.confidence_thresh = exp_params.confidence_thresh;
			self.x_max = exp_params.x_max;
			self.T_max = exp_params.T;
			self.T_tv  = exp_params.T_tv;
			self.lock_steps = exp_params.lock_steps;
			self.strategy_names = exp_params.strategy_names;

			self.a_lim = exp_params.controller.a_lim;
			self.dt = exp_params.dynamics.dt;
		end
		
		%% state_transition: State trainsition among all states in finite state machine
		function strategy_next = state_transition(self, exp_states)
			state_next = "N/A";
			strategy_next = "N/A";

			t = exp_states.t;
			EV_curr = exp_states.EV_curr;
			TV_pred = exp_states.TV_pred;
			score = exp_states.score;
			feas = exp_states.feas;
			actual_collision = exp_states.actual_collision;
			ref_col = exp_states.ref_col;


			switch self.state
				case "Start"
					state_next = "Free-Driving";
				case "Free-Driving"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toSafeCon(score, EV_curr, TV_pred)
						state_next = "Safe-Confidence";
						strategy_next = "Yield";
					elseif self.toSafeYield(score, EV_curr, TV_pred)
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					elseif self.toHOBCA(score, feas, EV_curr, TV_pred)
						state_next = "HOBCA-Unlocked";

						[~, max_idx] = max(score);
						strategy_tmp = self.strategy_names(max_idx);

						if strategy_tmp == "Yield"
							disp_msg = sprintf('%s: Yield is triggered when transitioning to HOBCA unlocked', self.state);
							error(disp_msg);
						else
							strategy_next = strategy_tmp;
						end
					else
						state_next = "Free-Driving";
					end
				case "Safe-Confidence"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toFD(t, EV_curr, TV_pred)
						state_next = "Free-Driving";
					elseif self.toEB(actual_collision)
						state_next = "Emergency-Break";
					elseif self.toSafeYield(score, EV_curr, TV_pred)
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					elseif self.toSafeInfeas(score, feas, EV_curr, TV_pred)
						state_next = "Safe-Infeasible";
						strategy_next = "Yield";
					elseif self.toHOBCA(score, feas, EV_curr, TV_pred)
						state_next = "HOBCA-Unlocked";

						[~, max_idx] = max(score);
						strategy_tmp = self.strategy_names(max_idx);

						if strategy_tmp == "Yield"
							disp_msg = sprintf('%s: Yield is triggered when transitioning to HOBCA unlocked', self.state);
							error(disp_msg);
						else
							strategy_next = strategy_tmp;
						end
					else
						state_next = "Safe-Confidence";
						strategy_next = "Yield";
					end
				case "Safe-Yield"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toFD(t, EV_curr, TV_pred)
						state_next = "Free-Driving";
					elseif self.toEB(actual_collision)
						state_next = "Emergency-Break";
					elseif self.toSafeCon(score, EV_curr, TV_pred)
						state_next = "Safe-Confidence";
						strategy_next = "Yield";
					elseif self.toSafeInfeas(score, feas, EV_curr, TV_pred)
						state_next = "Safe-Infeasible";
						strategy_next = "Yield";
					elseif self.toHOBCA(score, feas, EV_curr, TV_pred)
						state_next = "HOBCA-Unlocked";

						[~, max_idx] = max(score);
						strategy_tmp = self.strategy_names(max_idx);

						if strategy_tmp == "Yield"
							disp_msg = sprintf('%s: Yield is triggered when transitioning to HOBCA unlocked', self.state);
							error(disp_msg);
						else
							strategy_next = strategy_tmp;
						end
					else
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					end
				case "Safe-Infeasible"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toFD(t, EV_curr, TV_pred)
						state_next = "Free-Driving";
					elseif self.toEB(actual_collision)
						state_next = "Emergency-Break";
					elseif self.toSafeCon(score, EV_curr, TV_pred)
						state_next = "Safe-Confidence";
						strategy_next = "Yield";
					elseif self.toSafeYield(score, EV_curr, TV_pred)
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					elseif self.toHOBCA(score, feas, EV_curr, TV_pred)
						state_next = "HOBCA-Unlocked";

						[~, max_idx] = max(score);
						strategy_tmp = self.strategy_names(max_idx);

						if strategy_tmp == "Yield"
							disp_msg = sprintf('%s: Yield is triggered when transitioning to HOBCA unlocked', self.state);
							error(disp_msg);
						else
							strategy_next = strategy_tmp;
						end
					else
						state_next = "Safe-Infeasible";
						strategy_next = "Yield";
					end
				case "HOBCA-Unlocked"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toFD(t, EV_curr, TV_pred)
						state_next = "Free-Driving";
					elseif self.toSafeCon(score, EV_curr, TV_pred)
						state_next = "Safe-Confidence";
						strategy_next = "Yield";
					elseif self.toSafeYield(score, EV_curr, TV_pred)
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					elseif self.toSafeInfeas(score, feas, EV_curr, TV_pred)
						state_next = "Safe-Infeasible";
						strategy_next = "Yield";
					elseif self.toHOBCA_lock(score, ref_col, feas, EV_curr, TV_pred)
						state_next = "HOBCA-Locked";

						strategy_next = self.strategy;
					else
						state_next = "HOBCA-Unlocked";

						[~, max_idx] = max(score);
						strategy_tmp = self.strategy_names(max_idx);

						if strategy_tmp == "Yield"
							disp_msg = sprintf('%s: Yield is triggered when transitioning to HOBCA unlocked', self.state);
							error(disp_msg);
						else
							strategy_next = strategy_tmp;
						end
					end
				case "HOBCA-Locked"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toFD(t, EV_curr, TV_pred)
						state_next = "Free-Driving";
					elseif self.toSafeYield(score, EV_curr, TV_pred)
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					elseif self.toSafeInfeas(score, feas, EV_curr, TV_pred)
						state_next = "Safe-Infeasible";
						strategy_next = "Yield";
					else
						state_next = "HOBCA-Locked";

						strategy_next = self.strategy;
					end
				case "Emergency-Break"
					if self.toEnd(t, EV_curr)
						state_next = "End";
					elseif self.toEB(actual_collision)
						state_next = "Emergency-Break";
					% ========= Below are just for completing the experiments 
					elseif self.toFD(t, EV_curr, TV_pred)
						state_next = "Free-Driving";
					elseif self.toSafeCon(score, EV_curr, TV_pred)
						state_next = "Safe-Confidence";
						strategy_next = "Yield";
					elseif self.toSafeYield(score, EV_curr, TV_pred)
						state_next = "Safe-Yield";
						strategy_next = "Yield";
					elseif self.toSafeInfeas(score, feas, EV_curr, TV_pred)
						state_next = "Safe-Infeasible";
						strategy_next = "Yield";
					elseif self.toHOBCA(score, feas, EV_curr, TV_pred)
						state_next = "HOBCA-Unlocked";

						[~, max_idx] = max(score);
						strategy_tmp = self.strategy_names(max_idx);

						if strategy_tmp == "Yield"
							disp_msg = sprintf('%s: Yield is triggered when transitioning to HOBCA unlocked', self.state);
							error(disp_msg);
						else
							strategy_next = strategy_tmp;
						end
					else
						disp_msg = sprintf('%s: Unexpected Situation', self.state);
						error(disp_msg);
					end
						
					
				otherwise
					disp_msg = sprintf('Current state: %s is not recognized.', self.state);
					error(disp_msg);
			end

			self.state = state_next;
			self.strategy = strategy_next;

			fprintf('The Operation State is now %s, with strategy %s\n', self.state, self.strategy);

		end

		% ============= Transition Conditions

		%% toEnd: The transition criteria to End
		function output = toEnd(self, t, EV_curr)
			if EV_curr(1) >= self.x_max || t >= self.T_max - self.N
				output = true;
			else
				output = false;
			end
		end

		%% toEB: The transition criteria to Emergency Break
		function output = toEB(self, actual_collision)
			if actual_collision
				output = true;
			else
				output = false;
			end
		end
		

		%% toFD: The transition criteria to Free Driving
		function output = toFD(self, t, EV_curr, TV_pred)
			rel_state = TV_pred - EV_curr;

			% If every step along the horizon is far away from the maneuver zone
			% or the current relative x drives out of the collision radius
			% or there is no recorded TV data available
			if all( rel_state(1, :) > 20 ) || rel_state(1, 1) < -self.r || t > self.T_tv - self.N % CHANGE: get rid of abs
				output = true;
			else
				output = false;
			end

		end

		%% toSafeCon: The transition criteria to Safety Control - Confidence
		function output = toSafeCon(self, score, EV_curr, TV_pred)

			% Check the Maneuver Zone
			rel_state = TV_pred - EV_curr;
			if all( rel_state(1, :) > 20 ) || rel_state(1, 1) < -self.r
				output = false;
				return
			end

			% Compute the distance threshold for applying braking assuming max
			% decceleration is applied
			TV_v = norm(TV_pred(4:5, 1));
			TV_th = TV_pred(3, 1);

			EV_v = norm(EV_curr(4:5));
			EV_th = EV_curr(3);

			rel_vx = TV_v*cos(TV_th(1)) - EV_v*cos(EV_th);
			min_ts = ceil(-rel_vx/abs(self.a_lim(1))/self.dt); % Number of timesteps requred for relative velocity to be zero
			v_brake = abs(rel_vx)+[0:min_ts]*self.dt*self.a_lim(1); % Velocity when applying max decceleration
			brake_thresh = sum(abs(v_brake)*self.dt) + 5*self.r;  % Distance threshold for safety controller to be applied
			d = norm(TV_pred(1:2,1) - EV_curr(1:2), 2); % Distance between ego and target vehicles

			%% Max score
			[~, max_idx] = max(score);
			strategy_tmp = self.strategy_names(max_idx);
		
			% If all scores are below the confidence threshold 
			% or the argmax is "yield"
			if max(score) <= self.confidence_thresh && d <= brake_thresh
				output = true;
			else
				output = false;
			end
		end

		%% toSafeYield: The transition criteria to Safety Control - Yield
		function output = toSafeYield(self, score, EV_curr, TV_pred)

			% Check the Maneuver Zone
			rel_state = TV_pred - EV_curr;
			if all( rel_state(1, :) > 20 ) || rel_state(1, 1) < -self.r
				output = false;
				return
			end

			%% Max score
			[~, max_idx] = max(score);
			strategy_tmp = self.strategy_names(max_idx);

			% if max(score) > self.confidence_thresh && strategy_tmp == "Yield"
			if strategy_tmp == "Yield"
				output = true;
			else
				output = false;
			end
		end
		

		%% toSafeInfeas: The transition criteria to Safety Control - Infeasible
		function output = toSafeInfeas(self, score, feas, EV_curr, TV_pred)

			% Check the Maneuver Zone
			rel_state = TV_pred - EV_curr;
			if all( rel_state(1, :) > 20 ) || rel_state(1, 1) < -self.r
				output = false;
				return
			end

			[~, max_idx] = max(score);
			strategy_tmp = self.strategy_names(max_idx);

			if (max(score) > self.confidence_thresh) && (strategy_tmp ~= "Yield") && (~feas)
				output = true;
			else
				output = false;
			end
		end

		%% toHOBCA: The transition criteria to HOBCA - Unlocked
		% Need to implicitly check maneuver zone by the order in if-else structure
		function output = toHOBCA(self, score, feas, EV_curr, TV_pred)

			% Check the Maneuver Zone
			rel_state = TV_pred - EV_curr;
			if all( rel_state(1, :) > 20 ) || rel_state(1, 1) < -self.r
				output = false;
				return
			end

			[~, max_idx] = max(score);
			strategy_tmp = self.strategy_names(max_idx);
			
			% If max score is above the confidence threshold
			% and the argmax is "Left" or "Right"
			% and the current HOBCA status is feasible
			if (max(score) > self.confidence_thresh) && (strategy_tmp ~= "Yield") && feas
				output = true;
			else
				output = false;
			end

		end

		%% toHOBCA_lock: The transition criteria to HOBCA - Locked
		% Need to implicitly check maneuver zone by the order in if-else structure
		function output = toHOBCA_lock(self, score, ref_col, feas, EV_curr, TV_pred)
			
			% Check the Maneuver Zone
			rel_state = TV_pred - EV_curr;
			if all( rel_state(1, :) > 20 ) || rel_state(1, 1) < -self.r
				output = false;
				return
			end

			[~, max_idx] = max(score);
			strategy_tmp = self.strategy_names(max_idx);
			
			% If there are more than 3 steps along ref traj in collision buffer
			% and max score is above the confidence threshold
			% and the argmax is "Left" or "Right"
			% and the current HOBCA status is feasible
			if (sum(ref_col) >= self.lock_steps) && (max(score) > self.confidence_thresh) && (strategy_tmp ~= "Yield") && feas
				output = true;
			else
				output = false;
			end
		end

    end

end


