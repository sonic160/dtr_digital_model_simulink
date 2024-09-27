clear; clc; close all;

addpath('../digital_model_complete/')
addpath('../digital_model_kinematics/')
warning('off')


% Define parameters of the simulink model.
mdl_name = 'main3_armpi_fpv.slx';
load_system(mdl_name);

joint1_damping = 0;
joint2_damping = 0;
damp_pince = 1000;

% Generate the time stamps corrsponding to the simulation.
len_time_series = 1000; % Sequence length.
simulation_time = 10; % The time that this sequence corresponds to.
% Calculate the endtime of the sequence.    
end_time_value_in_seconds = (len_time_series-1)*simulation_time/len_time_series; % The time that the last point corresponds to.
% Get the time steps of the sequence.
time_stamps = 0:0.01:end_time_value_in_seconds;

% Define the simulaion set-up.
traj_type_list = {'random_move'}; % How many trajectory types to consider.
label_list = {'Healthy', 'Motor_1_Stuck', 'Motor_2_Stuck', 'Motor_3_Stuck', 'Motor_4_Stuck', 'Motor_5_Stuck'}; % Define the labels to be simulated.
n_label = length(label_list); % Get the number of labels.
n_traj_per_label = 400; % Define the number of trajectory simulated per label.
visualization = 0; % Turn off the visualization.

root_dir = 'RobotPdMDataset/';

% Start simulation loop.
% Loop over each class. Creat a subfolder per class.
for i = 1:n_label 
    % Retrive the label.
    label = label_list{i};
    failure_type = i-1; % Index for the class.

    % Check if the subfolder exists
    sub_folder = [root_dir label];
    if ~exist(sub_folder, 'dir')
        % If the folder does not exist, create it
        mkdir(sub_folder);
    end

    counter = 0; % Define a counter to name the data files in a given subfolder.
    
    % Loop over the required trajectory type.
    for j = 1:length(traj_type_list)
        % Get the current trajectory type.
        traj_type = traj_type_list{j};
        % Generate n_traj_per_label trajectories.
        input_motor_commands = generateTrajectories(traj_type, n_traj_per_label, time_stamps);

        % Simulate the needed trajectories.
        for k = 1:n_traj_per_label
            fprintf('i: %d/%d; j: %d/%d; k: %d/%d\n', i, n_label, j, length(traj_type_list), k, n_traj_per_label);

            % Generate filename
            counter = counter + 1;
            filePath = fullfile(sub_folder, ['dataset_' num2str(counter)]);

            % Run simulation
            [joint_cmds, joint_resps, traj_cmd, traj_resp] = run_simulation(input_motor_commands{k}, failure_type, mdl_name, visualization);
            
            % Save the trajectories as dataset.
            dataset = zeros(len_time_series, 6);
            dataset(:, 1:3) = traj_cmd;
            dataset(:, 4:6) = traj_resp;
            save(filePath, 'dataset');

            % Log also the command and response on the component level.
            hidden_dataset = {joint_cmds, joint_resps};
            save(fullfile(sub_folder, ['hidden_dataset_' num2str(counter)]), 'hidden_dataset');
        end
    end
    
end