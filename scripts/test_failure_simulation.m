clear; clc; close all;

addpath('../digital_model_complete/')
addpath('../digital_model_kinematics/')

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

% Generate a random pick operation.
traj_type = 'pick';
n_traj = 1;
input_motor_commands = generateTrajectories(traj_type, n_traj, time_stamps);

failure_type = 2; % No failure
visualization = 1;
[joint_cmds, joint_resps, traj_cmd, traj_resp] = run_simulation(input_motor_commands, failure_type, mdl_name, visualization);


