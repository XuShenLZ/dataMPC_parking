%% nn: neural network training
function [net,tr] = nn(x, t, trn_size, val_size, hidden_size, model_name)
	% Choose a Training Function
	% For a list of all training functions type: help nntrain
	% 'trainlm' is usually fastest.
	% 'trainbr' takes longer but may be better for challenging problems.
	% 'trainscg' uses less memory. Suitable in low memory situations.
	trainFcn = 'trainscg';  % Scaled conjugate gradient backpropagation

	% Create a Fitting Network
	hiddenLayerSize = hidden_size;
	net = fitnet(hiddenLayerSize,trainFcn);

	% Custom Properties
	net.trainParam.epochs = 2000;
	% net.trainParam.lr = 0.02;
	% For a list of all transfer/activation functions type: help nntransfer
	transferFcn = 'tansig'; % or 'poslin'
	for i = 1:length(hiddenLayerSize)
		net.layers{i}.transferFcn = transferFcn;
	end

	% Setup Division of Data for Training, Validation, Testing
	net.divideFcn = 'divideind';
	net.divideParam.trainInd = 1 : trn_size;
	net.divideParam.valInd   = (trn_size+1) : (trn_size+floor(0.5*val_size));
	net.divideParam.testInd  = (trn_size+floor(0.5*val_size)+1) : (trn_size+val_size);

	% Train the Network
	[net,tr] = train(net,x,t, ...
						'useParallel', 'yes', ...
						'useGPU', 'yes');

	% Test the Network
	% y = net(x);
	% e = gsubtract(t,y);
	% performance = perform(net,t,y)

	% View the Network
	view(net)

	% Plots
	% Uncomment these lines to enable various plots.
	figure, plotperform(tr)
	%figure, plottrainstate(tr)
	%figure, ploterrhist(e)
	%figure, plotregression(t,y)
	%figure, plotfit(net,x,t)

	%% Save the network as model
	uisave({'net', 'tr'}, ['../models/', model_name, ...
							'_TF-', trainFcn, ...
							'_h', num2str(hiddenLayerSize, '-%d'), ...
							'_AC-', transferFcn, ...
							'_ep', num2str(net.trainParam.epochs), ...
							'_ER', num2str(tr.best_perf), ...
							'_', datestr(now,'yyyy-mm-dd_HH-MM'), ...
							'.mat'])
