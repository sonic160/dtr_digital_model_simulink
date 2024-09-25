clear; clc; close all;

% Define parameters of the simulink model.
joint1_damping = 0;
joint2_damping = 0;
damp_pince = 1000;

% Generate the control commands.
len_time_series = 1000; % Sequence length.
% Let motor 1-5 to move from 0 to 240 degrees in 10 seconds.
input_motor_commands = cell(5);
for i = 1:5
    input_motor_commands{i} = linspace(0, 240, len_time_series);
end

% Transform the scales of the motor positions.
j1 = 180 + input_motor_commands{1};
j2 = -input_motor_commands{2};
j3 = input_motor_commands{3};
j4 = -input_motor_commands{4};
j5 = input_motor_commands{5};

end_time_value_in_seconds = (len_time_series-1)*0.01;     

% Generate control sequences.
joint1_ts = timeseries(j1/180*pi, 0:0.01:end_time_value_in_seconds);
joint2_ts = timeseries(j2/180*pi,0:0.01:end_time_value_in_seconds);
joint3_ts = timeseries(j3/180*pi,0:0.01:end_time_value_in_seconds);
joint4_ts = timeseries(j4/180*pi,0:0.01:end_time_value_in_seconds);
joint5_ts = timeseries(j5/180*pi,0:0.01:end_time_value_in_seconds);

% Define error flags.
m0 = [transpose(1:len_time_series), zeros(len_time_series, 1)]; % Indicator for failure operation.
m1 = [transpose(1:len_time_series), ones(len_time_series, 1)]; % Indicator for normal operation.

% Initial conditions: All motors are OK.
error1 = m1;
error2 = m1;
error3 = m1;
error4 = m1;
error5 = m1;
error6 = m1;