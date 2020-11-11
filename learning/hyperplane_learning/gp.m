function [trainedModel, val_MSE] = gp(trn_feature, trn_label, val_feature, val_label, response_id)
% Returns a trained regression model and its val_RMSE. This code recreates the
% model trained in Regression Learner app. Use the generated code to
% automate training the same model with new data, or to learn how to
% programmatically train models.

predictors = trn_feature';

response = trn_label(response_id, :)';

isCategoricalPredictor = repmat(false, 1, size(trn_feature, 1));

% Train a regression model
% This code specifies all the model options and trains the model.
regressionGP = fitrgp(...
    predictors, ...
    response, ...
    'BasisFunction', 'constant', ...
    'KernelFunction', 'exponential', ...
    'Standardize', true);

trainedModel = compact(regressionGP);

%% =============== Validation

% Compute validation predictions
validationPredictors = val_feature';
validationResponse = val_label(response_id, :)';
validationPredictions = predict(trainedModel, validationPredictors);

% Compute validation RMSE
isNotMissing = ~isnan(validationPredictions) & ~isnan(validationResponse);
val_MSE = nansum(( validationPredictions - validationResponse ).^2) / numel(validationResponse(isNotMissing) );

