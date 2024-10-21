In this notebook, we will generate training dataset using the RobotSimulator and TrajectoryGenerator classes. **Source code of this tutorial is available from: [scripts\generateTrainingData.mlx](../scripts/generateTrainingData.mlx).**

We will consider a blance dataset: For each label 0 - 8, we generate 400 trajectories, where 

   -  label 0 indicates no failure; 
   -  labels 1-4 indicate the stuck of motor 1-4, respectively; 
   -  labels 5-8 indicate steady-state error on motor 1-6, respectively. 

We assign the folder name to store the training data in variable `trainingDataFolderName`. The data will be saved in a folder named `trainingDataFolderName`, under the dictionary `../dataset/trainingDatasets/. ` Each label will be organized as a subfolder. You will find in each subfolder:

   -  dataset.mat: containing the trajCmd, trajResp and motorCmdsRadius, representing the desired, reliazed trajectories (x, y, z coordinates) and the commands of the five motors. 
   -  hidden_dataset.mat: containing motorResponsesRadius, representing the unobserved motor responses. 

There are some important parameters that defines the normal and failure behaviors:

   -  `robotSimulator.errorBlockSize, robotSimulator.errorValues, robotSimulator.errorProb, robotSimulator.holdingTimeIdx` define the errors on a normal motor: If set, the motor will have a delay of `robotSimulator.holdingTimeIdx`, and has a steady state error defined by a discrete state random variables with values robotSimulator.errorValues and probability distribution `robotSimulator.errorProb` 
   -  `robotSimulator.error_ll, robotSimulator.error_ul` define the lower and upper bounds of the steady-state error when simulating the steady-state error failure. The steady-state error will be a random intergal generated from the lower and upper bounds. 

```matlab:Code
clear; clc; close all;
warning('off');

% Set random seed for reproduction purpose.
rng(1000);

trainingDataFolderName = '20241017'; % Name of the training dataset.
pathtrainingSetup = ['../dataset/trainingDatasets/' 'trainingSetup' trainingDataFolderName '.mat']; % Path to the logged training setup.
% If the training setup exist, load it.
if isfile(pathtrainingSetup)
    load(pathtrainingSetup);
else   
    % Initialize the simulation objects.
    % Define the length of simulation and the corresponding sequence length.
    nTrajPerLabel = 400; % Define the number of trajectory simulated per label.
    lenSeries = 1000; % Sequence length.
    simulationTime = 10; % The time that this sequence corresponds to.
    % Instantiate a RobotSimulator class.
    robotSimulator = RobotSimulator(lenSeries, simulationTime);
    % Instantiate a TrajectoryGenerator class.
    trajGenerator = TrajectoryGenerator(lenSeries, simulationTime);
    
    % Parameters that defines motor accuracy.
    robotSimulator.errorBlockSize = 200; % The length of a error. The same error will persists for errorBlockSize points.
    % robotSimulator.errorValues = -3:3; % The possible errors on the positions, in robot control unit.
    % robotSimulator.errorProb = [.1, .1, .1, .4, .1, .1, .1]; % Probability distribution of different errors.
    % robotSimulator.holdingTimeIdx = 5; % The initial delay period of the motor.
    
    % Parameters related to the stuck failure.
    robotSimulator.stuckInstantIdx = 50;
    
    % Range of the steady-state errors.
    % robotSimulator.error_ll = 5;
    % robotSimulator.error_ul = 20;
    robotSimulator.error_ll = 10;
    robotSimulator.error_ul = 50;
    robotSimulator.visualization = 0; % Turn off the visualization.
    
    % Define the simulaion set-up.
    
    pathTrainingData = ['../dataset/trainingDatasets/' trainingDataFolderName '/'];
    trajTypeList = {'random_move'}; % How many trajectory types to consider.
    labelList = {'Healthy', 'Motor_1_Stuck', 'Motor_2_Stuck', 'Motor_3_Stuck', 'Motor_4_Stuck', ...
        'Motor_1_Steady_state_error', 'Motor_2_Steady_state_error', 'Motor_3_Steady_state_error', 'Motor_4_Steady_state_error'}; % Define the labels to be simulated.
    failureTypeList = 0:8;
    nLabel = length(labelList); % Get the number of labels.

    save(pathtrainingSetup);
end
```

```matlab:Code
tic;  % Start timer

% Start simulation loop.
% Loop over each class. Creat a subfolder per class.
for i = 1:nLabel 
    % Retrive the label.
    label = labelList{i};
    failureType = failureTypeList(i); % Index for the class.

    % Check if the subfolder exists
    subFolder = [pathTrainingData label];
    if ~exist(subFolder, 'dir')
        % If the folder does not exist, create it
        mkdir(subFolder);
    end

    counter = 0; % Define a counter to name the data files in a given subfolder.
    
    % Loop over the required trajectory type.
    for j = 1:length(trajTypeList)
        % Get the current trajectory type.
        trajGenerator.trajType = trajTypeList{j};
        % Generate n_traj_per_label trajectories.
        motorCmdsList = trajGenerator.generateTrajectories(nTrajPerLabel);
        
        clc;
        
        % Simulate the needed trajectories.
        for k = 1:nTrajPerLabel           
            % Generate filename
            counter = counter + 1;
            filePath = fullfile(subFolder, ['dataset_' num2str(counter)]);

            % Run simulation
            robotSimulator.runSimulation(motorCmdsList{k}, failureType);
            % Retrieve the results.
            trajCmds = robotSimulator.trajCommands;
            trajResps = robotSimulator.trajResponses;
            motorCmdsRadius = robotSimulator.motorCommandsRadius;
            motorRespsRadius = robotSimulator.motorResponsesRadius;

            if mod(k, 100)==1                
                fprintf('i: %d/%d; j: %d/%d; k: %d/%d\n', i, nLabel, j, length(trajTypeList), k, nTrajPerLabel);
                % robotSimulator.saveVisualizationFigs(motorCmdsRadius, motorRespsRadius, trajCmds, trajResps, subFolder, k);
            end
            
            % Save the trajectories as dataset.
            save(filePath, 'trajCmds', 'trajResps', 'motorCmdsRadius');
            % Log also the command and response on the component level.
            save(fullfile(subFolder, ['hidden_dataset_' num2str(counter)]), 'motorRespsRadius');
        end
    end
    
end
```

```text:Output
i: 1/9; j: 1/1; k: 1/400
i: 1/9; j: 1/1; k: 101/400
i: 1/9; j: 1/1; k: 201/400
i: 1/9; j: 1/1; k: 301/400
i: 2/9; j: 1/1; k: 1/400
i: 2/9; j: 1/1; k: 101/400
i: 2/9; j: 1/1; k: 201/400
i: 2/9; j: 1/1; k: 301/400
i: 3/9; j: 1/1; k: 1/400
i: 3/9; j: 1/1; k: 101/400
i: 3/9; j: 1/1; k: 201/400
i: 3/9; j: 1/1; k: 301/400
i: 4/9; j: 1/1; k: 1/400
i: 4/9; j: 1/1; k: 101/400
i: 4/9; j: 1/1; k: 201/400
i: 4/9; j: 1/1; k: 301/400
i: 5/9; j: 1/1; k: 1/400
i: 5/9; j: 1/1; k: 101/400
i: 5/9; j: 1/1; k: 201/400
i: 5/9; j: 1/1; k: 301/400
i: 6/9; j: 1/1; k: 1/400
i: 6/9; j: 1/1; k: 101/400
i: 6/9; j: 1/1; k: 201/400
i: 6/9; j: 1/1; k: 301/400
i: 7/9; j: 1/1; k: 1/400
i: 7/9; j: 1/1; k: 101/400
i: 7/9; j: 1/1; k: 201/400
i: 7/9; j: 1/1; k: 301/400
i: 8/9; j: 1/1; k: 1/400
i: 8/9; j: 1/1; k: 101/400
i: 8/9; j: 1/1; k: 201/400
i: 8/9; j: 1/1; k: 301/400
i: 9/9; j: 1/1; k: 1/400
i: 9/9; j: 1/1; k: 101/400
i: 9/9; j: 1/1; k: 201/400
i: 9/9; j: 1/1; k: 301/400
Warning: Some output might be missing due to a network interruption. To get the missing output, rerun the script.
```

```matlab:Code
elapsedTime = toc;  % End timer and save the elapsed time in a variable
disp(['Elapsed time: ', num2str(elapsedTime/3600), ' hours']);
```

```text:Output
Elapsed time: 8.0019 hours
```
