classdef RobotSimulator  < handle
% This class is used to control the robot following desired movements for
% motors 1-5.
% Important properties:
% - motorCommands: A cell array of 1*5. Each element is a timeseries
% object of the control command on motors 1-5, respectively. Each motor
% command takes values in [0, 1000], which will be translated to [0, 240]
% degrees in this function respectively.
% - motorResponses: A cell array of 1*5. The same format as motorCommands.
% 
% - failure_type: An integral that represents the failure types you want to
% simulate.

properties
    lenTimeSeries = 1000; % Sequence length
    simulationTime = 10; % The time that this sequence corresponds to.
    simulationTimeStamps; % The stamps used for simulation.

    % Define parameters of the simulink model.
    simulinkMdl = 'main3_armpi_fpv.slx';
    visualizationMdl = 'robot_model_visualization.slx'
    fkMdl = 'robot_model';
    joint1Damping = 0; % Damping for joint 1.
    joint2Damping = 0; % Damping for joint 2.
    dampPince = 1000; % Dampoing for the gripper.

    % Cmd and responses.
    motorCommands = cell(1, 5); % Initialize a list to store the input commands.
    motorResponses = cell(1, 5); % Initialize a list to store the responses.
    motorCommandsRadius = cell(1, 5); % Initialize a list to store the input commands, in degree.
    motorResponsesRadius = cell(1, 5); % Initialize a list to store the responses, in degree.
    trajCommands;
    trajResponses;
    

    failureType = 0; % Failure type. Integal between 0 - 8.
    visualization = 1; % If we want visualize the results of the simulation.

    % Define motor accuracy.
    errorValues = [0, 0]; % The possible errors on the positions.
    errorProb = [.5, .5];
    errorBlockSize = 200; % The error changes every x points.
    holdingTimeIdx = 2;

    % Parameters for the stuck failure.
    stuckInstantIdx = 50; % The moment when stuck failure occurs.

    % Define the upper and lower limits of the steady-state error.
    error_ll = 10; % In units of the robot.
    error_ul = 50; % In units of the robot.
end


methods

    function obj = RobotSimulator(lenTimeSeries, simulationTime)
    % Contructor function.
    
        % Set initial values for the simulation time.
        obj.lenTimeSeries = lenTimeSeries;
        obj.simulationTime = simulationTime;
        % Calculate the endtime of the sequence.    
        endTime = (lenTimeSeries-1)*simulationTime/lenTimeSeries; % The time that the last point corresponds to.
        % Get the time steps of the sequence.
        timeInterval = simulationTime/lenTimeSeries;
        obj.simulationTimeStamps = 0:timeInterval:endTime;
        
        % Set initial values for the traj cmd and responses.
        obj.trajCommands = zeros(lenTimeSeries, 3);
        obj.trajResponses = zeros(lenTimeSeries, 3);
        
        % Add paths of the simulation model.
        addpath('../digital_model_complete/');
        addpath('../digital_model_kinematics/');
        
        % Load the simulink model.
        load_system(obj.simulinkMdl);
        load_system(obj.fkMdl);

        % Initialize the simulink parameters.
        assignin('base', 'joint1_damping', obj.joint1Damping);
        assignin('base', 'joint2_damping', obj.joint2Damping);
        assignin('base', 'damp_pince', obj.dampPince);
    end
    
    
    function runSimulation(self, motorCmds, failureType)
    % This function runs a single simulation of the robot trajectory.
    % Inputs: 
    % - motorCmds: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % - failure_type: An integral that represents the failure types you want to
    % simulate.
    % Returns:
    % The results will be saved in the properties:
    % motorCommands, motorResponses, motorCommandsRadius nad motorResponsesRadius 
    
    % Depending on the value of failure_type, direct to the corresponding
    % subfunction.  

        % Save the original command.
        self.motorCommands = motorCmds;
        self.motorCommandsRadius = self.cmdsUnit2Radius(motorCmds);

        % Choose different simulation function based on the value of failure_type. 
        if failureType==0 % No failure.
            % Simulate the influence on the motor 
            motorRespsRadius = self.simulateMotorResponses(motorCmds);
        end
        
        if failureType>0 && failureType<5 % Motor stuck failure.
            faildedMotorIdx = failureType; % Identify the stucked motor.
            motorRespsRadius = self.simulateMotorStuck(motorCmds, faildedMotorIdx);            
        end
        
        if failureType>4 && failureType<9 % Steady-state error.
            faildedMotorIdx = failureType - 4;
            motorRespsRadius = self.simulateMotorSSError(motorCmds, faildedMotorIdx);
        end

        % Save the motor commands in both robot unit and radius.   
        self.motorResponsesRadius = motorRespsRadius;
        self.motorResponses = self.cmdsRadius2Unit(motorRespsRadius);
        
        % Apply the virtual sensor to measure the 3d coordinates.
        % For the command.
        self.trajCommands = self.virtualSensor(self.motorCommandsRadius);
        % For the response
        self.trajResponses = self.virtualSensor(self.motorResponsesRadius);
        
        if self.visualization
            % Show the motor cmds V.S. responses.
            self.visualizeMotorSimulationResults(self.motorCommandsRadius, self.motorResponsesRadius);
            
            % Animate the trajectory response.
            self.createAnimation(self.motorResponsesRadius);

            % Visualize the trajectory and responses.
            self.visualizeTrajCmdsResponses(self.trajCommands, self.trajResponses);            
        end
    end


    function coordinates = virtualSensor(self, motorPositions)
    % This function applies a virtual sensor through forward kinematics to get
    % the 3-d coordinate of the end-effector trajectory from the joint
    % positions.
    % It also include a random noise to simulate model and measurement
    % uncertainty.
    
        % Define the forward kinematics model.
        mdl = self.fkMdl;
        ik = simscape.multibody.KinematicsSolver(mdl);
        base = "robot_model/World/W";
        follower = "robot_model/gripper_base/F";
        addFrameVariables(ik,"gripper_base", "translation", base, follower);
        addFrameVariables(ik,"gripper_base", "rotation", base, follower);
        targetIDs = ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"] ;
        addTargetVariables(ik,targetIDs);
        outputIDs =["gripper_base.Translation.x";"gripper_base.Translation.y";...
            "gripper_base.Translation.z"];
        addOutputVariables(ik,outputIDs);
        
        % Retrieve the motor positions.
        j1 = motorPositions{1}.Data;
        j2 = motorPositions{2}.Data;
        j3 = motorPositions{3}.Data;
        j4 = motorPositions{4}.Data;
        j5 = motorPositions{5}.Data;
        
        % Initial values of the coordinates.
        coordinates = zeros(length(j1), 3);
        
        len_time_series = length(j1);
        for i = 1:len_time_series
            targets = [j1(i), j2(i), j3(i), j4(i), j5(i)];
            targets = targets*180/pi; % Tranform from radius to degree.
            [outputVec, ~] = solve(ik, targets);
            coordinates(i, :) = outputVec';
        end
    
    end


    function motorRespsRadius = simulateMotorSSError(self, inputMotorCmds, failedMotorIdx)
    % This function runs a single simulation of the robot trajectory when one
    % motor have steady-state error.
    % Inputs: 
    % - input_motor_commands: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % - failed_motor_idx: Index of the stucked motor.
    % Returns:
    % - motorRespsRadius: Timeseries objects that represents the responses
    % from the motors.           
    
        % Generate the failed control signals.
        motorCmds = inputMotorCmds;
    
        % Change the commands by injecting failure.
        motorCmds{failedMotorIdx} = self.injectSSError(inputMotorCmds{failedMotorIdx});
    
        % Run a normal simulation and get the outputs.
        motorRespsRadius = self.simulateMotorResponses(motorCmds);
    end
    
    
    function motorCmd = injectSSError(self, motorCmd)
        % This function inject a steady-state error.
        % Input: motorCmd: A timeseries of the original control sequence. Note:
        % Unitless, in [0, 1000].
        % Return: updated motorCmd.
    
        % Get the control sequence from the input of timeserires.
        motorCmdValues = motorCmd.Data;
        
        % Number of blocks
        numBlocks = floor(length(motorCmdValues) / self.errorBlockSize);    
        
        % Generate the random error.
        errors = self.error_ll + (self.error_ul - self.error_ll) * rand(1, numBlocks);
        % Iterate through the blocks to be updated and apply the error
        for i = 1:numBlocks
            error = errors(i);
            if rand() < .5 % 50% change having negative error.
                error = -1*error;
            end
    
            % Update the control command.
            start_index = (i-1)*self.errorBlockSize + 1;
            end_index = i*self.errorBlockSize;
            motorCmdValues(start_index:end_index) = motorCmdValues(start_index:end_index) + error;
        end
        
        % Return the updated j1
        motorCmd.Data = motorCmdValues;
    end



    function motorRespsRadius = simulateMotorStuck(self, inputMotorCmds, failedMotorIdx)
    % This function runs a single simulation of the robot trajectory when one
    % motor get stuck.
    % Inputs: 
    % - input_motor_commands: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % Returns:
    % - motorRespsRadius: 1*5 cell array, each element is a timeseries of
    % the motor responses, in radius.
    
        % First run a normal simulation and get the outputs.
        motorRespsRadius = self.simulateMotorResponses(inputMotorCmds);
        
        % Select the stucked motor and set its position to be stucked at the moment
        % of stuck.
        % Get the sequence.
        failedRespValues = motorRespsRadius{failedMotorIdx}.Data;
        % Replace the data.
        failedRespValues(self.stuckInstantIdx:end) = failedRespValues(self.stuckInstantIdx);
        motorRespsRadius{failedMotorIdx}.Data = failedRespValues;
    end
    
    
    function motorRespsRadius = simulateMotorResponses(self, inputMotorCmds)
    % This function simulate the responses of the five motors given their
    % control commands.
    % Inputs: 
    % - input_motor_commands: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % Returns:
    % - motorRespsRadius: 1*5 cell array, each element is a timeseries of
    % the motor responses, in radius.
        
        % Simulate the position errors due to motor accuracy.
        inputMotorCmds = self.addPositionError(inputMotorCmds);

        % Transform the scales of the motor positions.
        % Get the control command in the correct format.
        motorCmdsRadius = self.cmdsUnit2Radius(inputMotorCmds);
             
        % Generate control sequences in the simulink model.
        for i = 1:length(motorCmdsRadius)
            assignin('base', ['joint' num2str(i) '_ts'], motorCmdsRadius{i});
        end
        
        % Run the simulation and get the response of the five motors.
        out = sim(self.simulinkMdl);
        
        % Extract the outputs.
        motorRespsRadius = {out.j1_resp, out.j2_resp, out.j3_resp, ...
            out.j4_resp, out.j5_resp};
    end


    function modifiedMotorCmds = addPositionError(self, motorCmds)
    % This function adds a random noise to the control commands to simulate
    % the position errors.

        % The minimal value of holdingTimeIdx should be 2.
        if self.holdingTimeIdx < 2
            self.holdingTimeIdx = 2;
        end

        modifiedMotorCmds = motorCmds;

        % Define the length of each block
        blockSize = self.errorBlockSize;
        % Add random noise to each motor.
        for i = 1:length(motorCmds)
            motorCmd = motorCmds{i};
            cmdValues = motorCmd.Data;
            % Number of blocks
            numBlocks = floor(length(cmdValues) / blockSize);
            error = randsample(self.errorValues, numBlocks, true, self.errorProb);
            for j = 1:numBlocks                
                % Update the control command.              
                start_index = (j-1)*blockSize + 1;
                end_index = j*blockSize;
                cmdValuesBlock = cmdValues(start_index:end_index);

                % Add holdingTimeIdx of delay.
                if start_index==1
                    startValue = cmdValuesBlock(1);
                else
                    startValue = cmdValues(start_index-1);
                end

                % Get the plateau starting index and the target value.
                cmdDiff = cmdValuesBlock(2:end) - cmdValuesBlock(1:end-1);
                idxNonZero = find(cmdDiff, 1, "first");
                cmdDiff(1:idxNonZero-1) = -1;
                idxPlateau = find(cmdDiff, 1, "last"); % Last non zero element.
                targetValue = cmdValuesBlock(idxPlateau+1)+error(j);

                if idxPlateau
                    cmdValuesBlock(1:self.holdingTimeIdx-1) = startValue;
                    cmdValuesBlock(self.holdingTimeIdx:idxPlateau+1) = linspace(startValue, targetValue, idxPlateau-self.holdingTimeIdx+2);
                    cmdValuesBlock(idxPlateau+2:end) = targetValue;
                end

                cmdValues(start_index:end_index) = cmdValuesBlock;
                % cmdValues(start_index:end_index) = cmdValues(start_index:end_index) + error(j);
            end
            motorCmd.Data = cmdValues;
            modifiedMotorCmds{i} = motorCmd;
        end  
    end


    function cmdsInRadius = cmdsUnit2Radius(self, inputCmds)
    % This function transform the original motor command in the robot-side
    % format into the simulink format, in radius.
    % Input: inputCmds: 1*5 cell arrays, where each element is a timeseries
    % of command, in robot control units.
    % Return: cmsInRadius: 1*5 cell array, the same format as input, but
    % transformed scale.
    
        cmdsInRadius = cell(size(inputCmds));
        % Do a loop to transfer all the cmds.
        for i = 1:length(inputCmds)
            cmdsInRadius{i} = self.cmdUnit2Radius(inputCmds{i});
        end
    end


    function cmdInRadius = cmdUnit2Radius(~, inputCmd)
    % This function transform the original motor command in the robot-side
    % format into the simulink format, in radius.
    % The main steps: 
    % - Transform scales from an integral between [0, 1000], to [0, 240]
    % degrees.
    % - Check if the command goes outside the range.
    % - Align the origin by adding an offset.
    
        % Check if the command goes beyond the range.
        cmdValues = inputCmd.Data;
        cmdValues(cmdValues>1000) = 1000;
        cmdValues(cmdValues<0) = 0;        
        
        % Create an time series object for the transformed command.
        cmdInRadius = inputCmd;
        cmdInRadius.Data = (120 - cmdValues*240/1000)/180*pi;
    end


    function cmdsInUnit = cmdsRadius2Unit(self, inputCmds)
    % This function transform the original motor command in the robot-side
    % format into the simulink format, in radius.
    % Input: inputCmds: 1*5 cell arrays, where each element is a timeseries
    % of command, in robot control units.
    % Return: cmsInRadius: 1*5 cell array, the same format as input, but
    % transformed scale.
    
        cmdsInUnit = cell(size(inputCmds));
        % Do a loop to transfer all the cmds.
        for i = 1:length(inputCmds)
            cmdsInUnit{i} = self.cmdRadius2Unit(inputCmds{i});
        end
    end


    function cmdInUnit = cmdRadius2Unit(~, cmd)
    % This function transform the original motor command in the simulink
    % side to the scale and format of the robot side.
    
        % Read the original data. In degree.
        cmdValues = cmd.Data;
        cmdValues = cmdValues/pi*180;
        cmdValues(cmdValues>120) = 120;
        cmdValues(cmdValues<-120) = -120;
        
        % Create an time series object for the transformed command.
        cmdInUnit = cmd;
        cmdInUnit.Data = (120 - cmdValues)*1000/240;   
    end


    function visualizeTrajCmdsResponses(~, trajCmd, trajResp)
        f1 = figure;
        f1.Position = [10 10 900 1200];

        len_time_series = size(trajResp, 1);

        subplot(3, 1, 1)
        grid on;
        % Scatter plot with color gradient based on point index
        scatter3(trajCmd(1:10:end, 1), trajCmd(1:10:end, 2), trajCmd(1:10:end, 3), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        title('Desired trajectory');

        subplot(3, 1, 2)
        grid on;
        % Scatter plot with color gradient based on point index
        scatter3(trajResp(1:10:end, 1), trajResp(1:10:end, 2), trajResp(1:10:end, 3), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        title('Realized trajectory');

        subplot(3, 1, 3)
        grid on;
        % Scatter plot with color gradient based on point index
        scatter3(trajCmd(1:10:end, 1) - trajResp(1:10:end, 1), trajCmd(1:10:end, 2) - trajResp(1:10:end, 2), ...
            trajCmd(1:10:end, 3) - trajResp(1:10:end, 3), ...
            50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        title('Residual');


        f2 = figure;
        f2.Position = [10 10 900 600];
        
        title_label = {'x', 'y', 'z'};
        for i = 1:3
            subplot(2, 3, i)
            plot(trajCmd(:, i), '--r');
            hold on;
            plot(trajResp(:, i), '-k');
            xlabel('index')
            ylabel(['trajectory ' title_label{i} ' axis (m)'])            

            subplot(2, 3, i+3)
            plot(trajCmd(:, i)-trajCmd(:, i), '--r');
            hold on;
            plot(trajCmd(:, i)-trajResp(:, i), '-k');
            xlabel('index')
            ylabel(['residual ' title_label{i} ' axis (m)'])
        end
        legend('Commands', 'Responses', [100 10 120 20], 'Orientation', 'horizontal')
    end



    function visualizeMotorSimulationResults(~, commands, responses)
    % Create a Figure of the control commands V.S. response for each motor.
        f1 = figure;
        f1.Position = [10 10 900 600];
    
        for i = 1:5
            subplot(2, 3, i);
            plot(commands{i}, '--r');
            hold on
            plot(responses{i}, '-k');          
            xlabel('time/s')
            ylabel('Motor rotation (rads)');
            title(['Response of motor ' num2str(i)])
        end
        legend('Command', 'Response', [100 10 120 20], 'Orientation', 'horizontal')

        f2 = figure;
        f2.Position = [10 10 900 600];
    
        for i = 1:5
            subplot(2, 3, i);
            plot(commands{i}.Time, commands{i}.Data-commands{i}.Data, '--r');
            hold on
            plot(commands{i}.Time, commands{i}.Data-responses{i}.Data, '-k');          
            xlabel('time/s')
            ylabel('Residual (rads)');
            title(['Residual of motor ' num2str(i)])
        end
        legend('Command', 'Response', [100 10 120 20], 'Orientation', 'horizontal')
    end


    function createAnimation(self, responses)
        % Visualize the trajectory.
        for i = 1:length(responses)
            assignin('base', ['j' num2str(i) '_resp'], responses{i});
        end        
        % Run simulation to visualize the results.
        sim(self.visualizationMdl);
    end


    function saveVisualizationFigs(~, motorCmds, motorResps, trajCmd, trajResp, outputPath, datasetIdx)
        % Create a Figure of the control commands V.S. response for each motor.
        f1 = figure('Visible', 'off');
        f1.Position = [10 10 900 600];
    
        for i = 1:5
            subplot(2, 3, i);
            plot(motorCmds{i}, '--r');
            hold on
            plot(motorResps{i}, '-k');          
            xlabel('time/s')
            ylabel('Motor rotation (rads)');
            title(['Response of motor ' num2str(i)])
        end
    
        legend('Command', 'Response', [100 10 120 20], 'Orientation', 'horizontal')
        
        % Save the Figure.
        file_name = ['dataset_' num2str(datasetIdx) '_motors.png'];         % Define the file name   
        % Ensure the output directory exists
        if ~exist(outputPath, 'dir')
            mkdir(outputPath);
        end
        % Full path to save the plot
        full_file_path = fullfile(outputPath, file_name);
        % Save the figure
        saveas(f1, full_file_path);  % Save as PNG or any other format like .fig, .pdf, etc.
        % Close the figure to free up memory
        close(f1);
    
    
        f2 = figure('Visible', 'off');
        f2.Position = [10 10 900 600];
    
        for i = 1:5
            subplot(2, 3, i);
            plot(motorCmds{i}.Time, motorCmds{i}.Data-motorCmds{i}.Data, '--r');
            hold on
            plot(motorCmds{i}.Time, motorCmds{i}.Data-motorResps{i}.Data, '-k');          
            xlabel('time/s')
            ylabel('Residual (rads)');
            title(['Residual of motor ' num2str(i)])
        end
        legend('Command', 'Response', [100 10 120 20], 'Orientation', 'horizontal')
    
        file_name = ['dataset_' num2str(datasetIdx) '_motors_residuals.png'];         % Define the file name   
        % Full path to save the plot
        full_file_path = fullfile(outputPath, file_name);
        % Save the figure
        saveas(f2, full_file_path);  % Save as PNG or any other format like .fig, .pdf, etc.
        % Close the figure to free up memory
        close(f2);
    
        f3 = figure('Visible', 'off');
        f3.Position = [10 10 900 600];
        
        title_label = {'x', 'y', 'z'};
        for i = 1:3
            subplot(2, 3, i)
            plot(trajCmd(:, i), '--r');
            hold on;
            plot(trajResp(:, i), '-k');
            xlabel('index')
            ylabel(['trajectory ' title_label{i} ' axis (m)'])            
    
            subplot(2, 3, i+3)
            plot(trajCmd(:, i)-trajCmd(:, i), '--r');
            hold on;
            plot(trajCmd(:, i)-trajResp(:, i), '-k');
            xlabel('index')
            ylabel(['residual ' title_label{i} ' axis (m)'])
        end
        legend('Commands', 'Responses', [100 10 120 20], 'Orientation', 'horizontal')
        
        file_name = ['dataset_' num2str(datasetIdx) '_traj_residual.png'];         % Define the file name   
        % Full path to save the plot
        full_file_path = fullfile(outputPath, file_name);
        % Save the figure
        saveas(f3, full_file_path);  % Save as PNG or any other format like .fig, .pdf, etc.
        % Close the figure to free up memory
        close(f3);
    end


end

end