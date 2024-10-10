function input_motor_commands = generateTrajectories(traj_type, n_traj, time_stamps)
    % This function generate the command on the motor level by randomly
    % generated a trajectory of a given type.
    % Inputs:
    % - traj_type: str, 'pick' for pick-up-and-place. 
    % - n_traj: Number of trajectory needed.
    % Return:
    % - input_motor_commands: A cell array of 1*5. Each element is a
    % control signal of the given motor, in the format of timeseries.

    % Get the length of the desired command sequence.
    len_time_series = length(time_stamps);

    % Generate the trajectory.
    switch traj_type
        case 'random_move'
            zero_amount = 0;
            average_smallest_motive_lenght = 50;
            [trajectories, ~] = createRandomPickupList(n_traj, len_time_series, zero_amount, average_smallest_motive_lenght);
        otherwise
            error('traj_type invalid!')
    end

    % Prepare the desired control signals as a timeseries.
    input_motor_commands = cell(1, n_traj);
    for i = 1:n_traj
        tmp = cell(1, 5);
        for j = 1:5
            tmp{j} = inverse_transform_cmd_format(timeseries(trajectories{i}{j}, time_stamps));
        end
        input_motor_commands{i} = tmp;
    end
end


function joint_cmd_transformed = inverse_transform_cmd_format(input_motor_cmd)
    % This function transform the original motor command in the simulink
    % side to the scale and format of the robot side.
    
    % Read the original data. In degree.
    joint_cmd = input_motor_cmd.Data;
    joint_cmd(joint_cmd>120) = 120;
    joint_cmd(joint_cmd<-120) = -120;
    
    % Create an time series object for the transformed command.
    joint_cmd_transformed = input_motor_cmd;
    joint_cmd_transformed.Data = (120 - joint_cmd)*1000/240;   
end


function [trajectories, csv_file_equivalent] = createRandomPickupList(number_of_pickup_trajctories, len_time_series, zero_amount, average_smallest_motive_lenght)
    %_____
    %Creates a structure containing a wanted number of  random trajectories that mimic
    %a realistic pickup mouvement of a wanted length ( 5 commands, 1 for each
    %motor)
    %
    %   number_of_pickup_trajctories: number of trajectories that are given
    %   back
    %   len_time_series: number of points intrajectories that are given back
    %_____
    
    %Setting  default value to average_smallest_motive_lenght
    if nargin < 4
        average_smallest_motive_lenght=50; % Set a default value for average_smallest_motive_lenght
    end
    % Check if the condition is not met
    if len_time_series<average_smallest_motive_lenght
        error('Condition not met: len_time_series<average_smallest_motive_lenght, either reduce number of points for minimal motive or make the trajectory longer'); % Raise an error
    end
  
    max_number_of_motives = len_time_series/average_smallest_motive_lenght;
    
    % Initialize a cell array to store the trajectories
    trajectories = cell(1, number_of_pickup_trajctories);
    csv_file_equivalent = cell(1, number_of_pickup_trajctories);
    
    
    % Initialize counter
    generated_trajectories = 0;
    
    % Keep generating trajectories until the desired number is reached
    while generated_trajectories < number_of_pickup_trajctories        
        % Increment the counter
        generated_trajectories = generated_trajectories + 1;
        
        trajectory = cell(1, 5);
        triplets_cell = cell(1, 5);

        %Iterating over each of the 5 motors to create one trajectory
        allowed_amplitude = {80,50,50,50,50};
        danger_zone = false;
        for i = 1:4
            if danger_zone == true
                allowed_amplitude = {80,50,50,50,50};
            end
            % if i ==2
            %     [motor_command,triplets] = realisticsinglemotorcommand(max_number_of_motives,len_time_series,zero_amount, allowed_amplitude{i});
            % else
            %     [motor_command,triplets] = realisticsinglemotorcommandNOGENERATION(max_number_of_motives,len_time_series,zero_amount, allowed_amplitude{i});    
            % end
    
            [motor_command, triplets] = realisticsinglemotorcommand(max_number_of_motives,len_time_series,zero_amount, allowed_amplitude{i});
            
            if i == 1                
                if -50<triplets(1,1)<50
                    danger_zone = true;
                end
            end
            trajectory{i} = motor_command;
            triplets_cell{i} = triplets;
        end

        % Keep the 5th motor in its initial position, as it does not
        % impacts the end-effector.
        trajectory{5} = zeros(len_time_series, 1);
        number_of_motives_in_traj = 5;
        triplets = zeros(number_of_motives_in_traj, 3);
        triplets(:, 2:3) = 100*ones(number_of_motives_in_traj, 2);
        triplets_cell{5} = triplets;
    
        trajectories{generated_trajectories} = trajectory;
        csv_file_equivalent{generated_trajectories} = triplets_cell;       
    end
end


function [motor_command, triplets] = realisticsinglemotorcommand(max_number_of_motives, len_time_series, zero_amount, allowed_amplitude)
    %_______________
    %Returns the keypoints that are representative of a pickup movement and the command for the trajectory, these
    %can be used to make the real robot perform the trajectory. It is
    %considered that the robot is initially in a random position, will have to
    %move to a position to pick up an object and then move that object to a
    %final position.
    %
    %   n: case that is executed 
    %_______________
    
    % Point generation range (motor command amplitude)
    % From datasheet, the rotation speed of the motors is 272 degrees per
    % seconds, so 0.272 degree per ms, 2.72 per 10 ms.
    speed_cap = 2.72;
    
    % Random number of motives on the command of each motor
    % Note to self: maybe favorise appearance of 0 more often with better mechanism?
    
    % Initialising the points at 0
    motor_command = zeros(len_time_series, 1);
    
    % Choice of motor use or not
    percentage_zero_amount = zero_amount * 100;
    toggle_value = generateRandomNumbers(1, 100, 1);
    
    % Application of the toggle
    if toggle_value >= percentage_zero_amount        
        number_of_motives_in_traj = 5;
        triplets = zeros(number_of_motives_in_traj, 3);
    
        % Needs testing
        remaining_points = len_time_series;
        current_index = 0;
        old_point = 0;
        for i = 1:number_of_motives_in_traj
            % The two durations (one time top arrive at the point associated with the
            % motive and one for the plateau after attaining this point)
            
            % Inside motif arrival to plateau len rapport
            arrival_to_plateau_proportion = randi([3, 5]) / 10;
        
            % Motive length
            motive_len = 200;
        
            % Arrival to point length
            arrival_to_point_length = round(motive_len * arrival_to_plateau_proportion);
            triplets(i, 2) = arrival_to_point_length;
            if i ~= 1
                old_point = motor_command(current_index, 1);
            end 
            
            % Plateau length
            plateau_length = motive_len - arrival_to_point_length;
            triplets(i, 3) = plateau_length;
    
            % Randomly created next point            
            if rand() < 0.5
                point = rand() * allowed_amplitude;
            else                
                point =  - rand() * allowed_amplitude;
            end
            
            % Enforce boundary conditions
            min_point =  old_point - speed_cap * arrival_to_point_length;
            max_point = old_point + speed_cap * arrival_to_point_length;
            
            % Adjust the point to respect speed constraint
            if point < min_point
                point = min_point;
            elseif point > max_point
                point = max_point;
            end
            
            triplets(i, 1) = point;
    
            % Motor command updates
            motor_command(current_index + 1:current_index + arrival_to_point_length, 1) = linspace(old_point, point, arrival_to_point_length);
            motor_command(current_index + arrival_to_point_length + 1:current_index + motive_len, 1) = point;
    
            % Calculation of remaining points to be used 
            remaining_points = remaining_points - motive_len;
            current_index = len_time_series - remaining_points;
        end
    else
        triplets = [0, 0, 0];
        motor_command(1) = 0.0001;
        motor_command(1) = 0.0002;
    end
end


function randomIntegers = generateRandomNumbers(a, b, p)
    % Generate p random integers in the interval [a, b]
    randomIntegers = randi([round(a), round(b)], 1, p);
end