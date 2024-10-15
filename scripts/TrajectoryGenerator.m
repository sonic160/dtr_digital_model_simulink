classdef TrajectoryGenerator < RobotSimulator
% This is a class for generating random trajectories.
% It is inherented from RobotSimulator class.

properties
    trajType = 'random_move'; % Defines which type of trajectory to be generated.
    % Point generation range (motor command amplitude)
    % From datasheet, the rotation speed of the motors is 272 degrees per
    % seconds, so 0.272 degree per ms, 2.72 per 10 ms.
    speedCap = 2.72;
    % Number of movements in a generated trajectory.
    nMovementsPerTraj = 5;
    % Length of one movements (including the rising and plateau).
    movementLength = 200;
    % Define the range of the fraction of rising period in the movement.
    % The fraction will be generated as a random intergal from the range,
    % then translate to percentage by multiplying 10. So 3 means 30% of the
    % movement length.
    randomRangeRisingPeriod = [3, 5];
    allowedAmplitudeDegree = {80,50,50,50,50};
end

methods
    function motorCmdsLists = generateTrajectories(self, nTraj)
    % This function generate the command on the motor level by randomly
    % generated a trajectory of a given type.
    % Inputs:
    % - n_traj: Number of trajectory needed.
    % Return:
    % - motorCmdsLists: A cell array of 1*nTraj. Each element is a cell
    % array of 1*5, containing the control signals for the five motors.
       
        % Generate the trajectory.
        switch self.trajType
            case 'random_move'
                motorCmdsLists = self.generateCmdListRandomPickup(nTraj);
            otherwise
                error('traj_type invalid!')
        end
    end
    

    function motorCmdsList = generateCmdListRandomPickup(self, nTraj)
    % This function generate the command on the motor level by randomly
    % generated a trajectory of a given type.
    % Inputs:
    % - n_traj: Number of trajectory needed.
    % Return:
    % - motorCmdsLists: A cell array of 1*nTraj. Each element is a cell
    % array of 1*5, containing the control signals for the five motors.
    
        
        % Initialize a cell array to store the generated trajectories.
        motorCmdsList = cell(1, nTraj);        
        
        % Initialize counter
        counter = 0;        
        % Keep generating trajectories until the desired number is reached
        while counter < nTraj        
            % Increment the counter
            counter = counter + 1;
            
            % Initialize the motor commands.
            motorCmds = cell(1, 5);

            % Generate the commands for motors 1-4.
            for i = 1:4
                motorCmdDegree = self.generateMotorCmdRandomPickupTraj(i);
                motorCmds{i} = self.cmdDegree2Unit(motorCmdDegree);                
            end
    
            % Keep the 5th motor in its initial position, as it does not
            % impacts the end-effector.
            motorCmds{5} = timeseries(zeros(self.lenTimeSeries, 1), self.simulationTimeStamps);
            motorCmds{5} = self.cmdDegree2Unit(motorCmds{5});

            motorCmdsList{counter} = motorCmds;
        end
    end


    function motorCmdDegree = generateMotorCmdRandomPickupTraj(self, motorIndex)
        %_______________
        % This function generates a random pick-up movements: It comprises
        % of self.nMovementsPerTraj movements, where each movement
        % comprises of a rising period and a holding plateau. 
        % It assumes the motor starts from position 0.
        % Return: motorCmdDegree: A timeseries object of the control command.
        %_______________
        
        % Validity check
        if self.movementLength*self.nMovementsPerTraj > self.lenTimeSeries
            error('Generated sequence length must match the desired values!')
        end

        % Initialising the points at 0        
        motorCmdValues = zeros(self.lenTimeSeries, 1);
    
        % Keep generating the movements.
        remainingPoints = self.lenTimeSeries;
        currentIndex = 0;
        oldPoint = 0;
        for i = 1:self.nMovementsPerTraj
            % The two durations (one time top arrive at the point associated with the
            % motive and one for the plateau after attaining this point)
            
            % Inside motif arrival to plateau len rapport
            arrivalToPlateauProportion = randi(self.randomRangeRisingPeriod) / 10;      
            % Arrival to point length
            risingPeriodLength = round(self.movementLength * arrivalToPlateauProportion);
            
            % Get the prev points.
            if i ~= 1
                oldPoint = motorCmdValues(currentIndex, 1);
            end 
    
            % Randomly created next point            
            if rand() < 0.5
                point = rand() * self.allowedAmplitudeDegree{motorIndex};
            else                
                point =  - rand() * self.allowedAmplitudeDegree{motorIndex};
            end
            
            % Enforce boundary conditions
            minPoint =  oldPoint - self.speedCap * risingPeriodLength;
            maxPoint = oldPoint + self.speedCap * risingPeriodLength;
            
            % Adjust the point to respect speed constraint
            if point < minPoint
                point = minPoint;
            elseif point > maxPoint
                point = maxPoint;
            end
            
            % Motor command updates
            motorCmdValues(currentIndex + 1:currentIndex + risingPeriodLength, 1) = linspace(oldPoint, point, risingPeriodLength);
            motorCmdValues(currentIndex + risingPeriodLength + 1:currentIndex + self.movementLength, 1) = point;
    
            % Calculation of remaining points to be used 
            remainingPoints = remainingPoints - self.movementLength;
            currentIndex = self.lenTimeSeries - remainingPoints;
        end

        % Output the control cmd as a time series object.
        motorCmdDegree = timeseries(motorCmdValues, self.simulationTimeStamps);
    end

    
    function cmdInUnit = cmdDegree2Unit(~, cmd)
    % This function transform the original motor command in the simulink
    % side to the scale and format of the robot side.
    
        % Read the original data. In degree.
        cmdValues = cmd.Data;
        cmdValues(cmdValues>120) = 120;
        cmdValues(cmdValues<-120) = -120;
        
        % Create an time series object for the transformed command.
        cmdInUnit = cmd;
        cmdInUnit.Data = (120 - cmdValues)*1000/240;   
    end
    

end


end