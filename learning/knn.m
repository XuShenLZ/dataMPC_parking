function [trainedModel, val_acc] = knn(trn_feature, trn_label, val_feature, val_label, nn_k)

    % Extract predictors and response
    predictors = trn_feature';

    response = trn_label';

    isCategoricalPredictor = repmat(false, 1, size(trn_feature, 1));

    % Train a classifier
    % This code specifies all the classifier options and trains the classifier.
    classificationKNN = fitcknn(...
        predictors, ...
        response, ...
        'Distance', 'Euclidean', ...
        'Exponent', [], ...
        'NumNeighbors', nn_k, ...
        'DistanceWeight', 'Equal', ...
        'Standardize', true, ...
        'ClassNames', {'L'; 'R'; 'Y'});

    trainedModel = compact(classificationEnsemble);
    
    %% =============== Validation

    validationPredictors = val_feature';
    validationResponse = val_label';
    [validationPredictions, validationScores] = predict(trainedModel, validationPredictors);

    % Compute validation accuracy
    correctPredictions = strcmp( strtrim(validationPredictions), strtrim(validationResponse));
    isMissing = cellfun(@(x) all(isspace(x)), validationResponse, 'UniformOutput', true);
    correctPredictions = correctPredictions(~isMissing);
    val_acc = sum(correctPredictions)/length(correctPredictions);
