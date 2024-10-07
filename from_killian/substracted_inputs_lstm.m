cArray = simulated_cell_dataset';
sizearray = size(cArray);
numSeq = sizearray(1);

apply_function = @(matrix) subtract_lines_and_plot(matrix, 2);
modified_cArray = cellfun(apply_function, cArray, 'UniformOutput', false);

XTrain = modified_cArray;

numClasses = 9;
numRepeats = ceil(numSeq / numClasses);
YLabels = repmat(0:numClasses-1, 1, numRepeats);
YLabels = YLabels(1:numSeq);

YTrain = categorical(YLabels);

validationRatio = 0.2;
numValidation = round(validationRatio * numSeq);

XVal = XTrain(end-numValidation+1:end);
XTrain = XTrain(1:end-numValidation);

YVal = YTrain(end-numValidation+1:end);
YTrain = YTrain(1:end-numValidation);

miniBatchSize = 64;
inputSize = 3;
numHiddenUnits = 150;

layers = [
    sequenceInputLayer(inputSize, 'Name', 'inputFEN')
    bilstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    bilstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    convolution1dLayer(2, 5, 'Stride', 2, 'Padding', 1)
    maxPooling1dLayer(2, 'Stride', 3, 'Padding', 1)
    convolution1dLayer(5, 32, 'Padding', 'same', 'Stride', 2)
    globalAveragePooling1dLayer('Name', 'GlobalAveragePoolingfcn')
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
];

options = trainingOptions("adam", ...
    ExecutionEnvironment="gpu", ...
    GradientThreshold=1, ...
    MaxEpochs=360, ...
    MiniBatchSize=miniBatchSize, ...
    ValidationData={XVal,YVal}, ...
    ValidationFrequency=20, ...
    SequenceLength="longest", ...
    L2Regularization = 0.00005, ...
    Shuffle="once", ...
    Verbose=0, ...
    Plots="training-progress");

net = trainNetwork(XTrain, YTrain, layers, options);

save("2_class_differential", 'net');

YPred = predict(net, XVal);
[~, predictedClass] = max(YPred, [], 2);
categoricalPred = categorical(predictedClass - 1, 0:numClasses-1);

C = confusionmat(YVal, categoricalPred);

figure;
confusionchart(YVal, categoricalPred, 'RowSummary', 'row-normalized');
title('Confusion Matrix');

precision = diag(C) ./ sum(C, 1)';
recall = diag(C) ./ sum(C, 2);
f1Score = 2 * (precision .* recall) ./ (precision + recall);

disp('Class   Precision   Recall   F1 Score');
disp([transpose(1:size(C, 1)), precision, recall, f1Score]);

figure;
for i = 0:numClasses-1
    YTruBinary = (YVal == num2str(i));
    YpredROC = YPred(:, i+1);
    [X, Y, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1);
    plot(X, Y, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    hold on;
end
xlabel('False Positive Rate');
ylabel('True Positive Rate');
title('ROC Curves for Multi-Class Classification');
legend('show');

figure;
for i = 0:numClasses-1
    YTruBinary = (YVal == num2str(i));
    YpredROC = YPred(:, i+1);
    [precision, recall, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1, 'xCrit', 'reca', 'yCrit', 'prec');
    plot(recall, precision, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    hold on;
end
xlabel('Recall');
ylabel('Precision');
title('Precision-Recall Curves for Multi-Class Classification');
legend('Location', 'Best');
hold off;

function result_matrix = subtract_lines_and_plot(input_matrix, ~)
    if size(input_matrix, 1) ~= 6
        error('Input matrix must have 6 rows.');
    end
    result_matrix = input_matrix(4:6, :) - input_matrix(1:3, :);
end
