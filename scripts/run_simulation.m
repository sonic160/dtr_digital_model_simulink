function [joint_cmds, joint_resps, traj_cmd, traj_resp] = run_simulation(input_motor_commands, failure_type, mdl_name, visualization)
    % This function runs a single simulation of the robot trajectory.
    % Inputs: 
    % - input_motor_commands: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % - failure_type: An integral that represents the failure types you want to
    % simulate.
    % Returns:
    % - j1_resp - j5_resp: Timeseries objects that represents the responses
    % from the motors.
    
    % Depending on the value of failure_type, direct to the corresponding
    % subfunction.

    switch failure_type
        case 0
            [joint_cmds, joint_resps] = simulation_normal(input_motor_commands, mdl_name, visualization);
        otherwise
            if failure_type>0 && failure_type<6 % Motor stuck failure.
                failded_motor_idx = failure_type;
                [joint_cmds, joint_resps] = simulation_motor_stuck(input_motor_commands, failded_motor_idx, mdl_name, visualization);
            end        
    end
    
    % Apply the virtual sensor to measure the 3d coordinates.
    % For the command.
    traj_cmd = virtual_sensor(joint_cmds);
    % For the response
    traj_resp = virtual_sensor(joint_resps);
    
    if visualization
        f1 = figure;
        f1.Position = [10 10 900 1200];
    
        len_time_series = size(traj_resp, 1);
    
        subplot(3, 1, 1)
        grid on;
        % Scatter plot with color gradient based on point index
        scatter3(traj_cmd(1:10:end, 1), traj_cmd(1:10:end, 2), traj_cmd(1:10:end, 3), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        title('Desired trajectory');
    
        subplot(3, 1, 2)
        grid on;
        % Scatter plot with color gradient based on point index
        scatter3(traj_resp(1:10:end, 1), traj_resp(1:10:end, 2), traj_resp(1:10:end, 3), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        title('Realized trajectory');
    
        subplot(3, 1, 3)
        grid on;
        % Scatter plot with color gradient based on point index
        scatter3(traj_cmd(1:10:end, 1) - traj_resp(1:10:end, 1), traj_cmd(1:10:end, 2) - traj_resp(1:10:end, 2), ...
            traj_cmd(1:10:end, 3) - traj_resp(1:10:end, 3), ...
            50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        title('Residual');
    end

end


function coordinates = virtual_sensor(motor_positions)
    % This function applies a virtual sensor through forward kinematics to get
    % the 3-d coordinate of the end-effector trajectory from the joint
    % positions.
    % It also include a random noise to simulate model and measurement
    % uncertainty.
    
    % Define the forward kinematics model.
    mdl = 'robot_model';
    load_system(mdl)
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
    j1 = motor_positions{1}.Data;
    j2 = motor_positions{2}.Data;
    j3 = motor_positions{3}.Data;
    j4 = motor_positions{4}.Data;
    j5 = motor_positions{5}.Data;
    
    % Initial values of the coordinates.
    coordinates = zeros(length(j1), 3);
    
    len_time_series = length(j1);
    for i = 1:len_time_series
        targets = [j1(i), j2(i), j3(i), j4(i), j5(i)];
        targets = targets*180/pi;
        [outputVec, ~] = solve(ik, targets);
        coordinates(i, :) = outputVec';
    end

end


function [joint_cmds, joint_resps] = simulation_motor_stuck(input_motor_commands, failded_motor_idx, mdl_name, visualization)
    % This function runs a single simulation of the robot trajectory when one
    % motor get stuck.
    % Inputs: 
    % - input_motor_commands: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % - failed_motor_idx: Index of the stucked motor.
    % Returns:
    % - j1_resp - j5_resp: Timeseries objects that represents the responses
    % from the motors.
    
    % First run a normal simulation and get the outputs.
    [joint_cmds, joint_resps] = simulation_normal(input_motor_commands, mdl_name, 0);
    
    % Define the index when the stucked occurs.
    stuck_instant_idx = 50;
    % Select the stucked motor and set its position to be stucked at the moment
    % of stuck.
    % Get the sequence.
    failed_series = joint_resps{failded_motor_idx}.Data;
    % Replace the data.
    failed_series(stuck_instant_idx:end) = failed_series(stuck_instant_idx);
    joint_resps{failded_motor_idx}.Data = failed_series;
    
    % Visualize the data if asked.
    if visualization
        visualize_simulation_results(joint_cmds, joint_resps);
    end

end


function [joint_cmds, joint_resps] = simulation_normal(input_motor_commands, mdl_name, visualization)
    % This function runs a single simulation of the robot trajectory when no failure occurs..
    % Inputs: 
    % - input_motor_commands: A cell array of 1*5. Each element is a timeseries
    % object of the control command on motors 1-5, respectively. Each motor
    % command takes values in [0, 1000], which will be translated to [0, 240]
    % degrees in this function respectively.
    % Returns:
    % - j1_resp - j5_resp: Timeseries objects that represents the responses
    % from the motors.
    
    % Transform the scales of the motor positions.
    % Get the control command in the correct format.
    j1_cmd = transform_cmd_format(input_motor_commands{1});
    j2_cmd = transform_cmd_format(input_motor_commands{2});
    j3_cmd = transform_cmd_format(input_motor_commands{3});
    j4_cmd = transform_cmd_format(input_motor_commands{4});
    j5_cmd = transform_cmd_format(input_motor_commands{5});
    joint_cmds = {j1_cmd, j2_cmd, j3_cmd, j4_cmd, j5_cmd};
         
    % Generate control sequences.
    assignin('base','joint1_ts', j1_cmd);
    assignin('base','joint2_ts', j2_cmd);
    assignin('base','joint3_ts', j3_cmd);
    assignin('base','joint4_ts', j4_cmd);
    assignin('base','joint5_ts', j5_cmd);
    
    % Run the simulation and get the response of the five motors.
    out = sim(mdl_name);
    
    % Prepare the outputs.
    j1_resp = out.j1_resp;
    j2_resp = out.j2_resp;
    j3_resp = out.j3_resp;
    j4_resp = out.j4_resp;
    j5_resp = out.j5_resp;
    joint_resps = {j1_resp, j2_resp, j3_resp, j4_resp, j5_resp};
    
    if visualization
        visualize_simulation_results(joint_cmds, joint_resps);
    end

end


function visualize_simulation_results(commands, responses)
    % Create a Figure of the control commands V.S. response for each motor.
    f1 = figure;
    f1.Position = [10 10 900 600];

    for i = 1:5
        subplot(2, 3, i);
        plot(commands{i}, '--r');
        hold on
        plot(responses{i}, '-k');
        legend('Command', 'Response')
        xlabel('time/s')
        ylabel('Motor rotation (rads)');
        title(['Response of motor ' num2str(i)])
    end
    
    % Visualize the trajectory.
    assignin('base','j1_resp', responses{1});
    assignin('base','j2_resp', responses{2});
    assignin('base','j3_resp', responses{3});
    assignin('base','j4_resp', responses{4});
    assignin('base','j5_resp', responses{5});
    
    % Run simulation to visualize the results.
    sim('robot_model_visualization.slx');

end


function joint_cmd_transformed = transform_cmd_format(input_motor_cmd)
    % This function transform the original motor command in the robot-side
    % format into the simulink format.
    % The main steps: 
    % - Transform scales from an integral between [0, 1000], to [0, 240]
    % degrees.
    % - Check if the command goes outside the range.
    % - Align the origin by adding an offset.
    
    % Check if the command goes beyond the range.
    joint_cmd = input_motor_cmd.Data;
    joint_cmd(joint_cmd>1000) = 1000;
    joint_cmd(joint_cmd<0) = 0;
    
    
    % Create an time series object for the transformed command.
    joint_cmd_transformed = input_motor_cmd;
    joint_cmd_transformed.Data = (120 - joint_cmd*240/1000)/180*pi;

end