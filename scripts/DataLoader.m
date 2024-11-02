classdef DataLoader < RobotSimulator
% DataLoader is a class that defines utilities for loading, preprocessing
% the data for training the AI model, and utilites for evaluating the
% performance.
    properties
        shuffle = false; % If true: Shuffle the dataset when loading the data.
        readFromOriginalFiles = false; % If true: Always read from original files, not logged files.
        pathLoggedData; % Path of the saved dataset.
        pathDataset; % Path of the main folder that contains the dataset.
        pathLoggedTrainingData; % Path of the saved dataset.
        pathTrainingDataset; % Path of the main folder that contains the dataset.
        pathLoggedTestData; % Path of the saved dataset.
        pathTestDataset; % Path of the main folder that contains the dataset.
        pathOriginalTestData; % Path that stores original test data.
        nTrajPerLabelTestDataset = 10; % NUmber of trajs per each label in the test dataset.
        labelList = {'Healthy', ...
                'Motor_1_Stuck', 'Motor_2_Stuck', 'Motor_3_Stuck', 'Motor_4_Stuck', ...
                'Motor_1_Steady_state_error', 'Motor_2_Steady_state_error', 'Motor_3_Steady_state_error', 'Motor_4_Steady_state_error'};
        saveFigsInterval = 100;
    end

    methods
        % function obj = DataLoader(pathTrainingData, pathLoggedTrainingData)
        %     obj.pathLoggedData = pathLoggedTrainingData;
        %     obj.pathDataset = pathTrainingData;
        % end

        function [dataTables, y] = loadTrainingData(self)
            % Set the parameter values and call the corresponding function.
            self.pathDataset = self.pathTrainingDataset;
            self.pathLoggedData = self.pathLoggedTrainingData;
            [dataTables, y] = self.getData();
        end


        function X = extractFeaturesOriginal(~, X)
            % Get the orginal features: Motor commands and end-effector
            % trajectory responses.
            for index = 1:numel(X)
                tmp = X{index};
                % We drop the time, and the commands on the motor level.
                tmp = table2array(tmp(:, [2:5 10:12]));
                X{index} = tmp;
            end
        end


        function X = extractFeaturesTrajCmdTrajResidual(~, X)
            % Add an additional feature of the residual.
            for index = 1:numel(X)
                tmp = X{index};
                % We drop the time, and the commands on the motor level.
                tmp = table2array(tmp(:, 7:12));
                % Use residuals to replace response.
                residual = tmp(:, 1:3) - tmp(:, 4:6);
                tmp(:, 4:6) = residual;
                X{index} =  tmp;
            end
        end


        function [dataTables, y] = getData(self)
            % Check if the file exists
            if isfile(self.pathLoggedData) && ~self.readFromOriginalFiles
                % If the file exists, load the dataset
                load(self.pathLoggedData, 'dataTables', 'y');
                disp(['Loaded data from ' self.pathLoggedData]);
            else
                % If the file does not exist, run read_original_data
                try
                    [dataTables, y] = self.readOriginalData();
                catch
                    [dataTables, y] = self.readOriginalDataOldVersion();
                end
                % [dataTables, y] = self.readOriginalData();
                
                % Save X and y as 'conc_dataset.mat' under '../RobotPdMDataset'
                save(self.pathLoggedData, 'dataTables', 'y');
                disp(['Data generated and saved at ' self.pathLoggedData]);
            end
                        
            if self.shuffle
                % Shuffle data.
                % Get the number of elements in X (or y, since they are of the same length)
                numElements = numel(dataTables);
                % Generate a random permutation of indices from 1 to numElements
                randomOrder = randperm(numElements);
                % Shuffle X and y using the random permutation
                dataTables = dataTables(randomOrder);
                y = y(randomOrder);
            end

        end


        function [dataTables, y] = readOriginalData(self)
        % This function reads the original data under path mainFolder. 
        % It will return a dataTables that contain all the features, and a
        % cellarray y that contails the name of the labels.
        
            % Get the list of all subfolders
            subFolders = dir(self.pathDataset);
            subFolders = subFolders([subFolders.isdir]);  % Filter only directories
            subFolders = subFolders(~ismember({subFolders.name}, {'.', '..'})); % Remove '.' and '..'
            
            % Count how many dataset we have.
            n_dataset = 0;
            for i = 1:length(subFolders)
                % Locate the sub dictionary.
                folderPath = fullfile(self.pathDataset, subFolders(i).name);    
                % Get a list of all .mat files in the subfolder that start with 'dataset_'
                matFiles = dir(fullfile(folderPath, 'dataset_*.mat'));
                for j = 1:length(matFiles)
                    n_dataset = n_dataset + 1;
                end
            end
            
            % Initialize cell array for data and cell array for labels
            dataTables = cell(1, n_dataset);
            y = cell(1, n_dataset);
            
            % Iterate over each subfolder to retrieve the data.
            startIdx = 0;
            for i = 1:length(subFolders)
                fprintf('Extract the %d/%d labels.\n', i, length(subFolders));

                % Locate the sub dictionary.
                folderPath = fullfile(self.pathDataset, subFolders(i).name);    
                % Get a list of all .mat files in the subfolder that start with 'data_'
                matFiles = dir(fullfile(folderPath, 'dataset_*.mat'));
                
                % Loop through each valid .mat file                
                for j = 1:length(matFiles)
                    matFilePath = fullfile(folderPath, matFiles(j).name);

                    % Extract the number between '_' and '.mat'
                    numberStr = regexp(matFilePath, '(?<=_)\d+(?=\.mat)', 'match');
                    % Convert the extracted string to a number (optional)
                    idxOffset = str2double(numberStr{1});
                    idx = startIdx+idxOffset;
                    
                    if mod(idxOffset, self.saveFigsInterval)==0
                        dataTable = self.createDataTable(matFilePath, idxOffset);
                    else
                        dataTable = self.createDataTable(matFilePath);
                    end
                                        
                    % Append the loaded data to the cell array X
                    dataTables{idx} = dataTable;                    
                    % Append the subfolder name as the label to y
                    y{idx} = subFolders(i).name;
                end
                startIdx = startIdx + length(matFiles);
            end
        end

        
        function [dataTables, y] = readOriginalDataOldVersion(self)
        % This function reads the original data under path mainFolder. 
        % It will return a dataTables that contain all the features, and a
        % cellarray y that contails the name of the labels.
        
            % Get the list of all subfolders
            subFolders = dir(self.pathDataset);
            subFolders = subFolders([subFolders.isdir]);  % Filter only directories
            subFolders = subFolders(~ismember({subFolders.name}, {'.', '..'})); % Remove '.' and '..'
            
            % Count how many dataset we have.
            n_dataset = 0;
            for i = 1:length(subFolders)
                % Locate the sub dictionary.
                folderPath = fullfile(self.pathDataset, subFolders(i).name);    
                % Get a list of all .mat files in the subfolder that start with 'dataset_'
                matFiles = dir(fullfile(folderPath, 'dataset_*.mat'));
                for j = 1:length(matFiles)
                    n_dataset = n_dataset + 1;
                end
            end
            
            % Initialize cell array for data and cell array for labels
            dataTables = cell(1, n_dataset);
            y = cell(1, n_dataset);
            
            % Iterate over each subfolder to retrieve the data.
            startIdx = 0;
            for i = 1:length(subFolders)
                % Locate the sub dictionary.
                folderPath = fullfile(self.pathDataset, subFolders(i).name);    
                % Get a list of all .mat files in the subfolder that start with 'data_'
                matFiles = dir(fullfile(folderPath, 'dataset_*.mat'));
                hiddenDatasetFiles = dir(fullfile(folderPath, 'hidden_dataset_*.mat'));
                
                % Loop through each valid .mat file                
                for j = 1:length(matFiles)
                    matFilePath = fullfile(folderPath, matFiles(j).name);
                    hiddenDatasetPath = fullfile(folderPath, hiddenDatasetFiles(j).name);

                    % Extract the number between '_' and '.mat'
                    numberStr = regexp(matFilePath, '(?<=_)\d+(?=\.mat)', 'match');
                    % Convert the extracted string to a number (optional)
                    idxOffset = str2double(numberStr{1});
                    idx = startIdx+idxOffset;
                    
                    % Load the .mat file
                    loadedData = load(matFilePath);
                    dataset = loadedData.dataset;
                    
                    hidden_dataset = load(hiddenDatasetPath);
                    hidden_dataset = hidden_dataset.hidden_dataset;

                    motorCmdsRadius = hidden_dataset{1};
                    trajCmds = dataset(:, 1:3);
                    trajResps = dataset(:, 4:6);
                    
                    % Prepare the data values.
                    dataMatrix = zeros(size(trajCmds, 1), 12);
                    % Set timestamps.
                    dataMatrix(:, 1) = self.simulationTimeStamps;
                    % Set the motor commands.
                    for k = 1:5
                        dataMatrix(:, k+1) = motorCmdsRadius{k}.Data;
                    end
                    % Set the desired traj.
                    dataMatrix(:, 7:9) = trajCmds;
                    % Set the obtained traj.
                    dataMatrix(:, 10:12) = trajResps;
        
                    % Create a dataTable
                    colNames = {'Timestamps', 'Motor1Cmd', 'Motor2Cmd', 'Motor3Cmd', 'Motor4Cmd', 'Motor5Cmd', ...
                        'DesiredTrajectory-x', 'DesiredTrajectory-y', 'DesiredTrajectory-z', ...
                        'RealizedTrajectory-x', 'RealizedTrajectory-y', 'RealizedTrajectory-z'};
                    dataTable = array2table(dataMatrix, 'VariableNames', colNames);
                    dataTable.matFilePath = repmat({matFilePath}, height(dataTable), 1);

                                        
                    % Append the loaded data to the cell array X
                    dataTables{idx} = dataTable;
                    
                    % Append the subfolder name as the label to y
                    y{idx} = subFolders(i).name;
                end
                startIdx = startIdx + length(matFiles);
            end
        end



        function dataTable = createDataTable(self, matFilePath, datasetIdxSaveFigs)
        % Read a single .mat file and create a dataTable.
        % The dataTable has 12 columns:
        % - timestamps
        % - control commands for each motor (2-6)
        % - trajectory command (7-9)
        % - trajectory response (10-12)

            % Assign default value for datasetIdxSaveFigs: 0.
            % 0 means do not save Figs.
            if nargin == 2
                datasetIdxSaveFigs = 0;
            end

            % Load the .mat file
            loadedData = load(matFilePath);
            motorCmdsRadius = loadedData.motorCmdsRadius;
            trajCmds = loadedData.trajCmds;
            trajResps = loadedData.trajResps;
            
            % Prepare the data values.
            dataMatrix = zeros(size(trajCmds, 1), 12);
            % Set timestamps.
            dataMatrix(:, 1) = motorCmdsRadius{1}.Time;
            % Set the motor commands.
            for i = 1:5
                dataMatrix(:, i+1) = motorCmdsRadius{i}.Data;
            end
            % Set the desired traj.
            dataMatrix(:, 7:9) = trajCmds;
            % Set the obtained traj.
            dataMatrix(:, 10:12) = trajResps;

            % Create a dataTable
            colNames = {'Timestamps', 'Motor1Cmd', 'Motor2Cmd', 'Motor3Cmd', 'Motor4Cmd', 'Motor5Cmd', ...
                'DesiredTrajectory-x', 'DesiredTrajectory-y', 'DesiredTrajectory-z', ...
                'RealizedTrajectory-x', 'RealizedTrajectory-y', 'RealizedTrajectory-z'};
            dataTable = array2table(dataMatrix, 'VariableNames', colNames);
            dataTable.matFilePath = repmat({matFilePath}, height(dataTable), 1);

            % Save Figs.
            if datasetIdxSaveFigs>0
                % Get the path of the current dict.
                [rootPath, ~, ~] = fileparts(matFilePath);

                % Read the hiddendataset to get motor responses.
                hiddendatasetPath = [rootPath '/hidden_dataset_' num2str(datasetIdxSaveFigs) '.mat'];
                loadedData = load(hiddendatasetPath);
                motorRespsRadius = loadedData.motorRespsRadius;

                % Save the Figs.
                self.saveVisualizationFigs(motorCmdsRadius, motorRespsRadius, trajCmds, trajResps, rootPath, datasetIdxSaveFigs);
            end
        end


        function [dataTables, y] = loadTestingData(self)
            % Read the original test data.
            % If read from original test data in robot format: 
            % Process the original data. Create a folder in
            % self.pathTestData, containing the dataset in
            % Matlab formats. Then read them.
            % Otherwise directly read.

            % If there not processed yet, process the orignal data.
            if ~exist(self.pathTestDataset, 'dir')
                self.parseOriginalTestData();
            end

            % Set the reading path to that of the processed data.
            self.pathDataset = self.pathTestDataset;
            self.pathLoggedData = self.pathLoggedTestData;
            % Read the processed data.
            [dataTables, y] = self.getData();
        end


        function parseOriginalTestData(self)
            % Get the list of all files and folders in the directory
            allItems = dir(self.pathOriginalTestData);
            % Filter out the items that are not directories
            subfolders = allItems([allItems.isdir]);
            % Remove the '.' and '..' entries (current and parent directories)
            subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'}));
            
            % Do a loop to extract all the dataset.
            for i = 1:length(subfolders)
                fprintf('Extract tests for %d/%d labels\n', i, length(subfolders));
            
                % Define the path of the subfolder that contains the original data.
                pathLabelOriginalData = [self.pathOriginalTestData subfolders(i).name];
                
                % Prepare the output path.
                outputPathFull = [self.pathTestDataset '\' self.labelList{str2num(pathLabelOriginalData(end))+1}];
                % Check if the directory exists
                if ~exist(outputPathFull, 'dir')
                    % If it does not exist, create it
                    mkdir(outputPathFull);
                end
                
                % Extract and save the real test data.
                
                % For the stuck failures, we need to inject the failure manually.
                if i>1 && i<6 % Motor stuck failure.
                    stuckMotorIdx = i-1;
                    self.parseOneLabel(pathLabelOriginalData, outputPathFull, stuckMotorIdx);
                else % For other failure, it is simulated on the robot directly.
                    stuckMotorIdx = 0;
                    self.parseOneLabel(pathLabelOriginalData, outputPathFull, stuckMotorIdx);
                end
            end
        end


        function parseOneLabel(self, pathLabelOriginalData, outputpath, stuckMotorIdx)
            % This function process the three csv files output by the data collection
            % program. It extracts each run from the original data, and output a folder
            % with sub folders in Matlab.
            
            
            % Read the data Tables.
            [cmd, duration, position] = self.readOriginalTestdata(pathLabelOriginalData);
            
            % Do a loop to extract each run.
            n_runs = self.nTrajPerLabelTestDataset;
            
            clc;
            
            for i = 1:n_runs
                fprintf('Extract %d/%d tests\n', i, n_runs);
                % Define folder name.
                filePath = fullfile(outputpath, ['dataset_' num2str(i)]);
            
                % Prepare initial values.
                motorCmds = cell(1, 5); % Commands.
                motorResps = cell(1, 5); % Responses.
            
                % Get the data for the current run.
                [cmdCurrent, durationCurrent, positionCurrent] = self.extractCurrentRun(cmd, duration, position, i);
               
                for j = 1:5
                    motor_idx = ['motor_' num2str(7-j)];
                    [motorCmdsValues, motorRespsValues] = self.generateSeqByInterpl(cmdCurrent, durationCurrent, positionCurrent, motor_idx);
                    
                    % Post processing.
                    motorCmds{j} = timeseries(motorCmdsValues(:, 2), motorCmdsValues(:, 1));      
                    motorResps{j} = timeseries(motorRespsValues(:, 2), motorRespsValues(:, 1));                 
                end

                % Transform the scales of the motor positions.
                % Get the control command in the correct format.
                motorCmdsRadius = self.cmdsUnit2Radius(motorCmds);
                motorRespsRadius = self.cmdsUnit2Radius(motorResps);
            
                if stuckMotorIdx>0 && stuckMotorIdx<5 % Simulate the stuck failure.
                    % Select the stucked motor and set its position to be stucked at the moment
                    % of stuck.
                    % Get the sequence.
                    failed_series = motorRespsRadius{stuckMotorIdx}.Data;
                    % Replace the data.
                    failed_series(self.stuckInstantIdx:end) = failed_series(self.stuckInstantIdx);
                    motorRespsRadius{stuckMotorIdx}.Data = failed_series;
                end

            
                % Apply the virtual sensor to measure the 3d coordinates.
                % For the command.
                trajCmds = self.virtualSensor(motorCmdsRadius);
                % For the response
                trajResps = self.virtualSensor(motorRespsRadius);
                       
                % Save the trajectories as dataset.
                save(filePath, 'trajCmds', 'trajResps', 'motorCmdsRadius');    
                % Log also the command and response on the component level.
                save(fullfile(outputpath, ['hidden_dataset_' num2str(i)]), 'motorRespsRadius');
                % % Save the Figures.
                % self.saveVisualizationFigs(motorCmdsRadius, motorRespsRadius, trajCmds, trajResps, outputpath, i);
            end
        
        end


        function [cmd, duration, position] = readOriginalTestdata(~, pathLabelOriginalTestData)
            % This function read the three csv files obtained from the robot.
            % Then, it aligns the time scale by in the column 'time_since_start'.
            % Returns:
            % - cmd, duration, position: Table object with aligned time scales.
            
            % Read the data Tables.
            cmd = readtable(fullfile(pathLabelOriginalTestData, 'trajectory_monitoring_cmd.csv')); % Commands
            duration = readtable(fullfile(pathLabelOriginalTestData, 'trajectory_monitoring_cmd_duration.csv')); % Duration of each movement
            position = readtable(fullfile(pathLabelOriginalTestData, 'trajectory_monitoring_position.csv')); % Monitored positions.
            
            % Get the time origin.
            timestamp_cmd = cmd.timestamp;
            timestamp_duration = duration.timestamp;
            timestamp_position = position.timestamp;
            starting_time = min([min(timestamp_cmd), min(timestamp_duration), min(timestamp_position)]);
            
            % Subtract the starting time.
            timestamp_cmd = timestamp_cmd - starting_time;
            timestamp_duration = timestamp_duration - starting_time;
            timestamp_position = timestamp_position - starting_time;
            
            % Save to column 'time_since_start'
            cmd.time_since_start = timestamp_cmd;
            duration.time_since_start = timestamp_duration;
            position.time_since_start = timestamp_position;        
        end


        function [cmd_i, duration_i, position_i] = extractCurrentRun(self, cmd, duration, position, currentIdx)
            % This function extract the corresponding data for the ith run, from the
            % original data of cmd, duration and position.
            
            % Extract current command and duration.
            cmd_i = cmd(((currentIdx-1)*7+2):((currentIdx-1)*7+6), :);
            duration_i = duration(((currentIdx-1)*7+2):((currentIdx-1)*7+6), :);
            
            % Get the moment when the first control command was sent in each
            % sequence. This is 2+(i-1)*7th element in timestamp_cmd.
            command_starting_time = cmd{2+(currentIdx-1)*7, 'time_since_start'};
            
            % Find indices of elements in timestamp_position that are smaller than command_starting_time
            timestamp_position = position.time_since_start;
            valid_indices = find(timestamp_position < command_starting_time);
            valid_elements = timestamp_position(valid_indices);
            [sequence_starting_time, relative_index] = max(valid_elements);
            sequence_starting_time_index = valid_indices(relative_index);
            
            % Find the end of this sequence.
            sequence_ending_time = sequence_starting_time + self.simulationTime;
            valid_indices = find(timestamp_position > sequence_ending_time);
            valid_elements = timestamp_position(valid_indices);
            [~, relative_index] = min(valid_elements);
            sequence_ending_time_index = valid_indices(relative_index);
            
            % Extract the current position.
            position_i = position(sequence_starting_time_index:sequence_ending_time_index, :);
        
        end
        
        
        function [reference_positions, response_positions] = generateSeqByInterpl(self, cmd, cmdDuration, position, motor)
        % Ths function takes as input the data obtained from the robot, and create
        % a sequence following the format of the Matlab training script by
        % interpolation.
        % Inputs:
        % - cmd, duration, position: The three Table objects containing the
        % original cata. After aligning the time scale.
        % - motor: int, which motor needs to be extracted.
        % - seq_length: The length of the new sequence after the interpolation.
        % - Returns:
        % - reference_positions, response_positions: Timeseries object of the
        % command and response for a given motor.
        
            % Get the command and duration data for the given motor.
            motor_command = cmd.(motor);
            cmdDuration = cmdDuration.(motor);
            
            % Find the staring index in the position.
            cmd_send_time = cmd.time_since_start; % Time instants when the control cmd is send.
            start_index_pos = find(position.time_since_start < cmd_send_time(1), 1, 'last'); % Right before the control cmd is sent.
            % Mark the starting position of the position Table: Starting from the prev
            % moment when control cmd was sent.
            if isempty(start_index_pos)
                error('Please start monitoring before the control command is sent!');
            else
                starting_position = position{start_index_pos, motor}; % The position before cmd is sent.
                cmd_send_time = cmd_send_time - position.time_since_start(1); % Reset the time scale.
                resp_measure_time = position.time_since_start - position.time_since_start(1);
            end
            
            % Get a serires points for interpolation.
            % Format: Each row: [time, position] for the control cmd sequence.
            points = zeros(2*length(motor_command)+1, 2); % Initial values.
            points(1, :) = [cmd_send_time(1), starting_position]; % Initial position.
            % First command: A linear increase to command(1), in duration(1) time.
            points(2, :) = [cmd_send_time(1)+cmdDuration(1)/1000, motor_command(1)];
            % Add the other command.
            for idx = 2:length(motor_command)
                points((idx-1)*2+1, :) = [cmd_send_time(idx), motor_command(idx-1)];  %this places the target point
                points((idx-1)*2+2, :) = [cmd_send_time(idx)+cmdDuration(idx)/1000, motor_command(idx)]; %this creates an articial target representing the end of the plateau
            end
            % Add the last point: Hold the position to the current value until the end
            % of the position sequence.
            points(end, :) = [resp_measure_time(end), motor_command(end)];
            if points(end, 1) < points(end-1, 1)
                points = points(1:end-1, :);
            end
            
            % Genearte the control cmd by interpolation.
            x_values = points(:, 1); % Time.
            y_values = points(:, 2); % Cmd.
            if x_values(1)>0
                x_values = [0; x_values];
                y_values = [y_values(1); y_values];
            end
            
            for i = 2:length(x_values) - 1
                if x_values(i) == 0
                    x_values(i) = (x_values(i - 1) + x_values(i + 1)) / 2;
                end
            end
            
            % Genearte len_time_series points.
            interpolated_x = self.simulationTimeStamps; % From start to end_time, generate the required length.
            interpolated_y = interp1(x_values, y_values, interpolated_x);
            reference_positions = [interpolated_x', interpolated_y'];
            % reference_positions = reference_positions(reference_positions(:, 1)<=self.simulationTime, :);
            
            % Process for the response positions.
            response_positions = interp1(resp_measure_time, position.(motor), interpolated_x);
            for i = 2:length(response_positions)
                if isnan(response_positions(i))
                    response_positions(i) = response_positions(i-1);
                end
            end
            response_positions = [interpolated_x', response_positions'];
            % response_positions = response_positions(response_positions(:, 1)<=self.simulationTime, :);
        
        end            


        function [accuracy, precision, recall, f1Score] = evalPerf(~, yTest, yPred)
            % Return the accuracy of a multi-class classificaiton problem.
            % Return also the precision, recall and f1 score for each class.
            confMat = confusionmat(yTest, yPred);
            
            % Initialize vectors to store precision, recall, F1 score for each class
            numClasses = size(confMat, 1); % Number of classes
            precision = zeros(numClasses, 1);
            recall = zeros(numClasses, 1);
            f1Score = zeros(numClasses, 1);
            
            % Calculate accuracy
            accuracy = sum(diag(confMat)) / sum(confMat(:));
            
            % Loop over each class to calculate precision, recall, and F1 score
            for i = 1:numClasses
                TP = confMat(i, i);  % True Positives for class i
                FP = sum(confMat(:, i)) - TP;  % False Positives for class i
                FN = sum(confMat(i, :)) - TP;  % False Negatives for class i
            
                % Precision = TP / (TP + FP)
                precision(i) = TP / (TP + FP);
            
                % Recall = TP / (TP + FN)
                recall(i) = TP / (TP + FN);
            
                % F1 Score = 2 * (precision * recall) / (precision + recall)
                f1Score(i) = 2 * (precision(i) * recall(i)) / (precision(i) + recall(i));
            end
        end


        function [X, M, S] = standardization(~, X, M, S)
            % This function normalizes the features.
            % Input: X is an cell array of 1*n_data, whose elements are matrixs of
            % (n_seq, n_features).
            % Output: X: The features after transformation.
            % The dimension is transposed: (n_features, n_seq)
        
            if nargin == 2
                % Concatenate all the features. Creat a matrix of (n_seq*n_data,
                % n_features).
                allFeatures = cat(1, X{:});
            
                % Do the normalization.
                M = mean(allFeatures);
                S = std(allFeatures);
            end
            
            for index = 1:numel(X)
               X{index} =  ((X{index} - M)./S)';
            end
        end


        function X = downsamling(~, X, interaval)
            % This function downsample a timeseries every interval points. 
            % To improve training efficiency with shorter sequence.
            for i = 1:numel(X)
                X_tmp = X{i};
                X{i} = X_tmp(1:interaval:end, :);
            end
        end       


    end
end