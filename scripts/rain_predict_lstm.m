% Step 1num_time_series: Generate the dataset
%num_classes = 16;

% trained_AI_model='bi_lstm_v2traj_300_c_l_i_motorerror_000102030405_0123_1000.mat';
% trajectory_dataset_name="./cellArray3_100NewPidLineCircleInterp_m1234_e000201030405.mat";
numClasses =9
%trajectory_dataset_name=['./',trajectory_dataset_name];
%trajectory_dataset_name =["dataset_jpoint_realistic_traj_100_motorerror000205.mat"];
% Parameters

%struc=load(trajectory_dataset_name);
%cArray=struc.cellArray;


%cArray=struc.cellArray;
cArray = simulated_cell_dataset'




% cArray=struc;
sizearray = size(cArray);
numSeq = sizearray(1); % Number of sequences
disp(numSeq)
%length of the testingdata
test_len=100;

% Size treatment
numberofcells=numel(cArray);
maxsize = size(cArray{1}, 2);
multfactor = maxsize / test_len;

% Create a new cell array to store modified data
modifiedCellArray = cArray;
disp(modifiedCellArray)


% Generate the pattern
pattern = mod(0:numSeq-1, numClasses);
% Create the categorical sequence
categoricalSequence = categorical(pattern, 0:numClasses-1);
% Repeat each category in categoricalSequence by multfactor times
repeatedSequence = categoricalSequence
disp('made it here 2')

modifiedCellArray = modifiedCellArray(~cellfun(@isempty, modifiedCellArray));
totalElements = numel(repeatedSequence);
indexToKeep = round(0.8 * totalElements);
totalCells = numel(modifiedCellArray);

index = round(0.8 * totalCells);
indexToKeep = index
XTrain = modifiedCellArray(1:index);
XVal = modifiedCellArray(index+1:totalCells);
YTrain = repeatedSequence(1:indexToKeep);

%XTrain = transition_remover(XTrain)
%XVal = transition_remover(XVal)
YVal =repeatedSequence(indexToKeep+1:totalElements);


miniBatchSize = 64;
% Step 2: Define the neural network

inputSize = 6;
numHiddenUnits = 150;



layers = [
    sequenceInputLayer(inputSize, 'Name', 'inputFEN')
    bilstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    bilstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    convolution1dLayer(2,5,'Stride',2,'Padding',1)
    maxPooling1dLayer(2,'Stride',3,'Padding',1)
    convolution1dLayer(5, 32, 'Padding', 'same', 'Stride', 2)
    globalAveragePooling1dLayer('Name', 'GlobalAveragePoolingfcn')
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
];

options = trainingOptions("adam", ...
    ExecutionEnvironment="gpu", ...
    GradientThreshold=1, ...InitialLearnRate=0.001, ... % Lower initial learning rate
    InitialLearnRate=0.001, ... % Lower initial learning rate
    MaxEpochs=600, ...
    MiniBatchSize=miniBatchSize, ...
    ValidationData={XVal,YVal}, ... %new
    ValidationFrequency=20, ...     %new
    SequenceLength="longest", ...
    L2Regularization = 0.01, ...  %new
    Shuffle="once", ...
    Verbose=0, ...
    Plots="training-progress");



net = trainNetwork(XTrain,YTrain,layers,options);



save('new_gen_trained_model.mat','net')
% Make predictions on the validation set
YPred = predict(net, XVal);

% Find the column index of the maximum probability for each row
[~, predictedClass] = max(YPred, [], 2);

% Create a categorical array from the predicted class indices
categoryNames = cellstr(num2str((0:max(predictedClass))'));  % Assuming classes are 0-based
categoricalPred = categorical(predictedClass - 1, 0:max(predictedClass), categoryNames);

% Compute confusion matrix
C = confusionmat(YVal, categoricalPred)

% Display confusion chart
figure
confusionchart(YVal, categoricalPred,'RowSummary','row-normalized');
title('Confusion Matrix');


precision = diag(C) ./ sum(C, 1)';
recall = diag(C) ./ sum(C, 2);
f1Score = 2 * (precision .* recall) ./ (precision + recall);


% Display the results
disp('Class   Precision   Recall   F1 Score');
disp([transpose(1:size(C, 1)), precision, recall, f1Score]);

% Create a new figure for ROC curves
figure

for i = 0:numClasses-1
    % Convert true labels to binary
    YTruBinary = ismember(YVal, num2str(i));
    
    % Extract predicted scores for the current class
    %YPredBinary =ismember(categoricalPred, num2str(i));
    YpredROC=YPred(:,i+1);
    
    % Compute ROC curve
    [X, Y, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1);
    
    % Plot ROC curve for the current class
    plot(X, Y, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    
    hold on;
end

% Add labels and legend
xlabel('False Positive Rate');
ylabel('True Positive Rate');
title('ROC Curves for Multi-Class Classification');
legend('show');

% Create a new figure for Precision-Recall curves
figure

for i = 0:numClasses-1
    % Convert true labels to binary
    YTruBinary = ismember(YVal, num2str(i));
    
    % Extract predicted scores for the current class
    YpredROC = YPred(:, i+1);
    
    % Compute Precision-Recall curve
    [precision, recall, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1, 'xCrit', 'reca', 'yCrit', 'prec');
    
    % Plot Precision-Recall curve for the current class
    plot(recall, precision, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    
    hold on;
end
% Add labels and legend
xlabel('Recall');
ylabel('Precision');
title('Precision-Recall Curves for Multi-Class Classification');
legend('Location', 'Best');
hold off; % Stop holding onto the current plot


%%

% Assuming 'cArray' is already loaded or defined as a 6D input dataset.
cArray = simulated_cell_dataset';
sizearray = size(cArray);
numSeq = sizearray(1); % Number of sequences

% Keep the 6D input (no subtracting)
XTrain = cArray;

% Define number of classes
numClasses = 9;

% Create categorical labels for training
numRepeats = ceil(numSeq / numClasses);
YLabels = repmat(0:numClasses-1, 1, numRepeats);
YLabels = YLabels(1:numSeq);

YTrain = categorical(YLabels);

% Split into training and validation sets (80% train, 20% validation)
validationRatio = 0.2;
numValidation = round(validationRatio * numSeq);

XVal = XTrain(end-numValidation+1:end);
XTrain = XTrain(1:end-numValidation);

YVal = YTrain(end-numValidation+1:end);
YTrain = YTrain(1:end-numValidation);

% Model parameters
miniBatchSize = 64;
inputSize = 6; % Keeping the 6D input
numHiddenUnits = 150;

% Define the neural network layers
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

% Training options
options = trainingOptions("adam", ...
    ExecutionEnvironment="gpu", ...
    GradientThreshold=1, ...
    MaxEpochs=300, ...  % Adjusted to match the second example
    MiniBatchSize=miniBatchSize, ...
    ValidationData={XVal,YVal}, ...
    ValidationFrequency=20, ...
    SequenceLength="longest", ...
    L2Regularization = 0.00005, ... 
    Shuffle="once", ...
    Verbose=0, ...
    Plots="training-progress");

% Train the network
net = trainNetwork(XTrain, YTrain, layers, options);

% Save the trained model
save("6D_input_trained_model.mat", 'net');

% Make predictions on the validation set
YPred = predict(net, XVal);

% Find the column index of the maximum probability for each row
[~, predictedClass] = max(YPred, [], 2);

% Create a categorical array from the predicted class indices
categoricalPred = categorical(predictedClass - 1, 0:numClasses-1);

% Compute confusion matrix
C = confusionmat(YVal, categoricalPred);

% Display confusion chart
figure;
confusionchart(YVal, categoricalPred, 'RowSummary', 'row-normalized');
title('Confusion Matrix');

% Calculate precision, recall, and F1 score
precision = diag(C) ./ sum(C, 1)';
recall = diag(C) ./ sum(C, 2);
f1Score = 2 * (precision .* recall) ./ (precision + recall);

% Display the results
disp('Class   Precision   Recall   F1 Score');
disp([transpose(1:size(C, 1)), precision, recall, f1Score]);

% Plot ROC curves
figure;
for i = 0:numClasses-1
    % Convert true labels to binary
    YTruBinary = (YVal == num2str(i));
    
    % Extract predicted scores for the current class
    YpredROC = YPred(:, i+1);
    
    % Compute ROC curve
    [X, Y, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1);
    
    % Plot ROC curve for the current class
    plot(X, Y, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    hold on;
end
xlabel('False Positive Rate');
ylabel('True Positive Rate');
title('ROC Curves for Multi-Class Classification');
legend('show');

% Plot Precision-Recall curves
figure;
for i = 0:numClasses-1
    % Convert true labels to binary
    YTruBinary = (YVal == num2str(i));
    
    % Extract predicted scores for the current class
    YpredROC = YPred(:, i+1);
    
    % Compute Precision-Recall curve
    [precision, recall, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1, 'xCrit', 'reca', 'yCrit', 'prec');
    
    % Plot Precision-Recall curve for the current class
    plot(recall, precision, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    hold on;
end
% Add labels and legend
xlabel('Recall');
ylabel('Precision');
title('Precision-Recall Curves for Multi-Class Classification');
legend('Location', 'Best');
hold off;

%%

function newCellArray = transition_remover(originalCellArray)
    % Check if the input is a cell array
    if ~iscell(originalCellArray)
        error('Input must be a cell array.');
    end
    
    % Initialize the new cell array to hold 6x80 matrices
    newCellArray = cell(size(originalCellArray));
    
    % Loop through each matrix in the original cell array
    for i = 1:numel(originalCellArray)
        matrix = originalCellArray{i};
        
        % Check if the matrix is 8x1000
        if size(matrix, 1) ~= 8 || size(matrix, 2) ~= 50
            error('Each matrix in the cell array must be 6x100.');
        end
        
        % Remove the first 20 columns
        newCellArray{i} = matrix(:, 5:end);
    end
end

