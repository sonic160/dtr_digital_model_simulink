%Examples of training a given AI model with a given dataset: 
%----------------------------------------------------------------------

%Used dataset:'premade_example_trajectory_dataset_interpolations_5_motors1_errors_0001'(this small example dataset comes already generated with the
%git normally, training anything with this small amount of data is amibitious)
%Name of the file that stores model architecture: 'rain_predict_lstm.m'
%(default value)
%Name of the model that will be trained: 'example_bi_lstm_5_i_motorerror_0001_1_1000.mat'
%processing_simulation(0,'training',1,'trajectory_dataset_name','premade_example_trajectory_dataset_interpolations_5_motors1_errors_0001', 'name_trained_AI_model','example_bi_lstm_5_i_motorerror_0001_1_1000.mat')


%Examples of simulated trajectory dataset generation then training a given
%---------------------------------------------------------------------------------------------------------
%AI model this dataset datasets: (classical use of this file)
%---------------------------------------------------------------------------------------------------------

%Lines: 100, Circles:100, Interpolations: 100 (default values used) / motors:1,2,3 / error types: normal/off/stutter/lag/speed-cap/steady-state
%Name of the file that stores model architecture: 'rain_predict_lstm.m'
%(default value)
%Name of the model that will be trained: 'example_bi_lstm_300_c_l_i_motorerror_000102030405_0123_1000.mat'
%processing_simulation(1,'training',1,'trajectory_dataset_name','example_trajectory_dataset_lines_100_circles_100_interpolations_100_motors123_errors_000102030405','circles',1,'lines',1,'interpolations',1,'name_trained_AI_model','example_bi_lstm_300_c_l_i_motorerror_000102030405_0123_1000.mat')

%____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________ 
%____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________  




function []= processing_simulation(varargin)
%--------------------------------------------------------------------------
%
%Function description:
%"""""""""""""""""""""
%   This single functions let us combine the entire process of our code
%   into a single run, bringing together the processes of: 
%       -generating a trajectory dataset that follows given characteristics
%       -training an AI model for a given type of architecture with this dataset
%    However this function is modulable: you can either only generate and
%    save a trajectory dataset that follows your specifications, or only train
%    and AI model with a dataset of your choice (it can have been generated
%    before hand), or do both. How to manipulate these options is descirbed
%    in the 'Potential inputs' description.
%
%Potential inputs:
%"""""""""""""""""  
%
% ''''''''''''''''''''''
% Most important inputs:
% ''''''''''''''''''''''
% 'simulating': this first argument is required (it is the only required argument) either 0 for no simulated
%  trajectory dataset generation or 1 for generation
%  the smallest possible function call is therfore: processing_simulation(1)
%
% 'training': either 0 for no AI training or 1 for the opposite
%  example of use: processing_simulation(1,'training',1)
%  default value: 0
%
% 'trajectory_dataset_name': the generated
% simulated trajectory dataset will be recorded under this name (don't use
% argument of you want to use default name or are not genertaing dataset)
%  default value: "placeholder_generated_dataset_name"
%
% 'trained_AI_model': the AI architecture that you will train will be
% determined by this file
% simulated trajectory dataset will be recorded under this name (don't use
% argument of you want to use default name or are not training AI)
%  default value: 'rain_predict_lstm.m'
%
% 'name_trained_AI_model': the trained AI model will be saved under this name (don't use
% argument of you want to use default name or are not genertaing dataset)
%  default value: 'trained_AI_model.m'
%
% 'circles':  either 0 for no circles in generated trajectory dataset or 1 for the opposite
%  example of use: processing_simulation(1,'circles',1)
%  default value: 0
%
% 'circle_number': number of circles in generated trajectory dataset
% example of use: processing_simulation(1,'circles',1, 'circle_number',2)
%  default value: 100
%
% 'lines': either 0 for no lines in generated trajectory dataset or 1 for the opposite
%  example of use: processing_simulation(1,'lines',1)
%  default value: 0
%
% 'line_number': number of lines in generated trajectory dataset
% example of use: processing_simulation(1,'lines',1, 'line_number',2)
%  default value: 100
%
% 'interpolations': either 0 for no interpolations in generated trajectory dataset or 1 for the opposite
%  example of use: processing_simulation(1,'interpolations',1)
%  default value: 0
%
% 'interpolation_number': number of interpolations in generated trajectory dataset
% example of use: processing_simulation(1,'interpolations',1, 'interpolations_number',2)
%  default value: 100
%
% 'realistic_trajs': either 0 for no realistic trajectories ( 1 motor command for each motor that can be decomposed into intervals where it is affine then constant with a plateau after each slope )in generated trajectory dataset or 1 for the opposite
%  example of use: processing_simulation(1,'realistic_trajs',1) 
%  default value: 0
%
% 'realistic_trajs_number': number of  realistic trajectories in generated trajectory dataset
% example of use: processing_simulation(1,'realistic_trajs',1, 'realistic_trajs_number',2)
%  default value: 100
%
% 'selection_erreures_moteur': selections of motors 1/2/3/4/5 and motor error
% types 00/01/02/03/04/05:
%   00: no error
%   01: [1,2,3,4,5] one of the respective motors (in order) is turned off 
%   02: [6,7,8,9,10] one of the respective motors (in order) will stutter
%   03: [11,12,13,14,15] one of the respective motors (in order) will lag
%   04: [16,17,18,19,20] one of the respective motors (in order) will have a speed-cap
%   05: [21,22,23,24,25] one of the respective motors (in order) will have a
%   steady-state error
% example of use: processing_simulation(1,'interpolations',1, 'interpolations_number',2,'selection_erreures_moteur',[0,1,2,8,12,13,15])
%  default value: [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25]
%
%
% '''''''''''''''''
% Secondary inputs:
% ''''''''''''''''''
%
%
%Outputs:
%""""""""
%--------------------------------------------------------------------------
    %Parsing the intput arguments to enable the setup of default values
    
    % Create an input parser
    p = inputParser;
    
    % Define optional input arguments with default values
    addRequired(p, 'simulating', @isnumeric); 
    addOptional(p, 'training',0, @isnumeric);
    addOptional(p, 'trajectory_dataset_name',"placeholder_generated_dataset_name",@(x) ischar(x) || isstring(x));
    addOptional(p, 'circles', 0, @isnumeric);
    addOptional(p, 'circle_number', 100, @isnumeric);
    addOptional(p, 'lines', 0, @isnumeric);
    addOptional(p, 'line_number', 100, @isnumeric);
    addOptional(p, 'interpolations', 0, @isnumeric);
    addOptional(p, 'interpolation_number', 100, @isnumeric);
    addOptional(p, 'realistic_trajs', 0, @isnumeric);
    addOptional(p, 'realistic_trajs_number', 100, @isnumeric);
    addOptional(p, 'selection_erreures_moteur', [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25], @isnumeric);
    addOptional(p, 'model_name', 'main3_armpi_fpv', @(x) ischar(x) || isstring(x));
    addOptional(p, 'distance_charac_robot', 0.28, @isnumeric);%décrit la zone dans laquelle le robot peut agir, par défault prend la v aleur spécifique du cas de notre étude
    addOptional(p, 'speedcap', 0.2, @isnumeric);
    addOptional(p, 'zero_amount', 0.0, @isnumeric);
    addOptional(p, 'average_smallest_motive_lenght', 50, @isnumeric);
    addOptional(p, 'joint1_damping', 0, @isnumeric);
    addOptional(p, 'joint2_damping', 0, @isnumeric);
    addOptional(p, 'damp_pince', 1000, @isnumeric);
    % addOptional(p, 'mdl', 'main3_armpi_fpv', @(x) ischar(x) || isstring(x));
    % addOptional(p, 'base', 'main3_armpi_fpv\World\W', @(x) ischar(x) || isstring(x));
    % addOptional(p, 'follower', 'main3_armpi_fpv\gripper_base\F', @(x) ischar(x) || isstring(x));
    addOptional(p, 'mdl', "robot_model", @(x) ischar(x) || isstring(x));
    addOptional(p, 'base', "robot_model/World/W", @(x) ischar(x) || isstring(x));
    addOptional(p, 'follower', "robot_model/gripper_base/F", @(x) ischar(x) || isstring(x));
    addOptional(p, 'targetIDs', ["gripper_base.Translation.x";"gripper_base.Translation.y";...
        "gripper_base.Translation.z"], @(x) ischar(x) || isstring(x));
    addOptional(p, 'outputIDs', ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"], @(x) ischar(x) || isstring(x));  
    addOptional(p, 'guessesIDs', ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"], @(x) ischar(x) || isstring(x)); 
    addOptional(p, 'guesses', [3,3,3,3,3], @isnumeric); 
    %addOptional(p, 'training', 0, @isnumeric);
    addOptional(p, 'trained_AI_model','rain_predict_lstm.m', @(x) ischar(x) || isstring(x));
    addOptional(p, 'name_trained_AI_model','trained_AI_model.m', @(x) ischar(x) || isstring(x));
    %%% PAS ENCORE PLEINEMENT FONCTIONNEL, N AFFECTE QUE INTERPOLATION POUR L INSTANT
    addOptional(p, 'nb_points_traj', 1000, @isnumeric);
    addOptional(p, 'stationary_error',20 , @isnumeric);
    addOptional(p, 'stationary_error_timestap', 100, @isnumeric);
    % Parse the input arguments
    parse(p, varargin{:});

    % Retrieve the values
    disp("-------------------")
    simulating = p.Results.simulating;
    fprintf('Il y aura simulation Y/N: %d.\n',simulating);
    trajectory_dataset_name = p.Results.trajectory_dataset_name;
    fprintf('Trajectory dataset name is:  %s.\n',trajectory_dataset_name);
    circles = p.Results.circles;
    circle_number = p.Results.circle_number;
    fprintf('Trajectory: circles Y/N: %d amount: %d.\n',circles,circle_number);
    lines = p.Results.lines;
    line_number = p.Results.line_number;
    fprintf('Trajectory: lines Y/N: %d amount: %d.\n',lines,line_number);
    interpolations = p.Results.interpolations;
    interpolation_number = p.Results.interpolation_number;
    fprintf('Trajectory: interpolation Y/N: %d amount: %d.\n',interpolations,interpolation_number);
    realistic_trajs = p.Results.realistic_trajs;
    realistic_trajs_number = p.Results.realistic_trajs_number;
    fprintf('Trajectory: realistic_trajs Y/N: %d amount: %d.\n', realistic_trajs,realistic_trajs_number);
    motorerrorselection = p.Results.selection_erreures_moteur;
    strmot=num2str(motorerrorselection);
    global num_classes
    num_classes= numel(motorerrorselection);
    assignin('base','num_classes',  num_classes);
    %fprintf('Les erreures moteur seront : %d m.\n',motorerrorselection);
    fprintf('Les erreures moteur seront : [%s%d]\n', sprintf('%d,', motorerrorselection(1:end-1)),motorerrorselection(end));
    training = p.Results.training;
    fprintf('Il y aura entrainement Y/N: %d.\n',training);
    trained_AI_model = p.Results.trained_AI_model;
    assignin('base','trained_AI_model',  trained_AI_model);
    name_trained_AI_model = p.Results.name_trained_AI_model;
    assignin('base','name_trained_AI_model',  name_trained_AI_model)
    if trajectory_dataset_name == "placeholder_generated_dataset_name"
        fprintf("!!!!ATTENTION NOT NAMING SIMULATED TRAJECTORY DATASET IS BAD PRACTICE, we recommend this name: simulated_trajectory_dataset_c_%d_%d_l_%d_%d_i_%d_%d_motor_errors_%s.mat \n",circles,circle_number,lines,line_number,interpolations,interpolation_number,strmot);
    end
    if training
        fprintf('L architecture du modèle entrainé sera régie par le fichier: %s.\n',trained_AI_model);
        fprintf('Le nom du modèle entrainé sera: %s.\n',name_trained_AI_model);
        if name_trained_AI_model == "trained_AI_model.m"
            fprintf("!!!!ATTENTION NOT NAMING TRAINED AI MODEL IS BAD PRACTICE, we recommend this name: trained_AI_model_insert/architecture/details/here_c_%d_%d_l_%d_%d_i_%d_%d_motor_errors_%s.mat \n",circles,circle_number,lines,line_number,interpolations,interpolation_number,strmot);
        end
        if isequal(motorerrorselection,[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25])
            fprintf("!!!!ATTENTION BASE CASE MOTORSELECTION : [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25] is used, (number of classes that the model will be trained to distinguish == numel(motorselection) MAKE SURE THIS IS INTENDED");
        end
        fprintf('Le nombre de classes pour l entrainement du model sera : %d.\n',num_classes);
    end

    %%déclaration globlale
    global model_name;
    model_name = p.Results.model_name;
    fprintf('Le modèle de robot simulé est : %s .\n',model_name);
    distance_charac_robot = p.Results.distance_charac_robot;
    fprintf('Le bras du robot pleinement étendu mesure: %f m.\n',distance_charac_robot);
    nb_points_traj = p.Results.nb_points_traj;
    fprintf('Le nombre de points par trajecoire est: %d points.\n',nb_points_traj);
    speedcap = p.Results.speedcap;
    fprintf('Le speedcap est: %d.\n',speedcap);
    disp("-------------------")
    zero_amount = p.Results.zero_amount;
    fprintf('Le zero_amount dans realistic_traj: %d.\n',zero_amount);
    disp("-------------------")
    average_smallest_motive_lenght = p.Results.average_smallest_motive_lenght;
    fprintf('Le average_smallest_motive_lenght dans realistic_traj: %d.\n',average_smallest_motive_lenght);
    disp("-------------------")
    stationary_error = p.Results.stationary_error;
    fprintf('Le stationary_error est: %d.\n',stationary_error);
    disp("-------------------")
    stationary_error_timestap = p.Results.stationary_error_timestap;
    fprintf('Le stationary_error_timestap est: %d.\n',stationary_error_timestap);
    disp("-------------------")


    %Creating the various trajectory datasets for the wanted shape types

    %Initialisation
    firsttype=0;
    secondtype=0;
    thirdtype=0;
    len_time_series=nb_points_traj;
    if circles || lines
        reduced_adapted_circle_set=CreateRandomCircleList(distance_charac_robot,circle_number);
        reduced_adapted_line_set=CreateRandomLineList(distance_charac_robot,line_number);
        reduced_adapted_shape_set=mergeStructures(reduced_adapted_circle_set,reduced_adapted_line_set);
        firsttype=1;
    end

    if interpolations
        reduced_adapted_interpolate_set= createInterpolate(interpolation_number, nb_points_traj);
        secondtype=1;
    end
    if realistic_trajs
        [realistic_trajs_set,realistic_trajs_representative_point_set]= createRandomPickupList(realistic_trajs_number,nb_points_traj, zero_amount,average_smallest_motive_lenght);
        for i = 1:5

      %  disp("point commands for motor " + string(i))
       % disp(realistic_trajs_representative_point_set{1,1}{1,i})

        end

        thirdtype=1;
    end
    if simulating
       load_system(model_name);
    end

    global joint1_damping;
    joint1_damping = p.Results.joint1_damping;
    assignin('base','joint1_damping',joint1_damping )
    fprintf('Le joint1_damping est: %d.\n',joint1_damping);
    global joint2_damping;
    joint2_damping = p.Results.joint2_damping;
    assignin('base','joint2_damping',joint2_damping )
    fprintf('Le joint1_damping est: %d.\n',joint2_damping);
    global damp_pince;
    damp_pince = p.Results.damp_pince;
    assignin('base','damp_pince', damp_pince)
    fprintf('Le damp_pince est: %d.\n',damp_pince);
    global mdl;
    mdl = p.Results.mdl;
    assignin('base','mdl', mdl)
    fprintf('Le modèle de robot est: %s.\n',mdl);
    load_system(mdl)
    global ik;
    ik = simscape.multibody.KinematicsSolver(mdl);
    assignin('base','ik', ik)
    global base;
    base = p.Results.base;
    assignin('base','base', base)
    fprintf('La base est: %s.\n',base);
    global follower;
    follower = p.Results.follower;
    assignin('base','follower', follower)
    fprintf('La follower est: %s.\n',follower);
    addFrameVariables(ik,"gripper_base","translation",base,follower);
    addFrameVariables(ik,"gripper_base","rotation",base,follower);
    global targetIDs;
    targetIDs = p.Results.targetIDs;
    assignin('base','targetIDs', targetIDs)
    fprintf('Les targetIDs sont: %s.\n',targetIDs);
    addTargetVariables(ik,targetIDs);
    global outputIDs;
    outputIDs = p.Results.outputIDs;
    assignin('base','outputIDs', outputIDs)
    fprintf('Les outputIDs sont: %s.\n',outputIDs);
    addOutputVariables(ik,outputIDs);
    global guessesIDs;
    guessesIDs = p.Results.guessesIDs;
    assignin('base','guessesIDs', guessesIDs)
    fprintf('Les guessesIDs sont: %s.\n',guessesIDs);
    global guesses;
    guesses = p.Results.guesses;
    assignin('base','guesses', guesses)
    fprintf('Les guesses sont : [%s%d]\n', sprintf('%d,', guesses(1:end-1)),guesses(end));
    addInitialGuessVariables(ik,guessesIDs);
    global datapoint;
    
    global simOut;
    
    %put all the variables in the workspace 

    
    %simul length: len_time_series/100= legnth of simulation in seconds

    scale_factor = rand();
    fprintf('Le scale facotr est: %d.\n', scale_factor);

    timescale=10/len_time_series;
    j1 = zeros(len_time_series,1);
    j2 = zeros(len_time_series,1);
    j3 = zeros(len_time_series,1);
    j4 = zeros(len_time_series,1);
    j5 = zeros(len_time_series,1);
    T = 10; % period
    spline = zeros(len_time_series,3);
    assignin('base','spline', spline)
    targets = zeros(len_time_series,3);
    assignin('base','targets', targets)

    %Creating dataset


    m0=[transpose(1:len_time_series), zeros(len_time_series, 1)];
    m1=[transpose(1:len_time_series), ones(len_time_series, 1)];

    if simulating && firsttype ~= 1 && secondtype ~= 1 && thirdtype ~= 1
        error('Please set either firsttype or secondtype or thirdtype to 1.');
    end
    
    dataset=[];
    scale_factor = rand();
    if firsttype
        dataset_1=[];
        shapes_dict=reduced_adapted_shape_set;
        shapelist=fieldnames(shapes_dict);
        numberofshapes=numel(shapelist);
        fprintf('The number of fields is:%d\n',numberofshapes);
        for k = 1:numberofshapes
            if k==floor(numberofshapes/4)
                disp("------------------------")
                disp("K =1/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes/2)
                disp("------------------------")
                disp("K =1/2 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes*3/4)
                disp("------------------------")
                disp("K =3/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            
        
            shape=shapelist{k};
            disp("------------------------")
            fprintf('Shape name:%s\n',shape);
            disp("------------------------")
        
        
            trajectory_motor_command=cell(1, 5);
            for t = 1:len_time_series
                t_echantillon=t/500;
                datapoint =[shapes_dict.(shape).xequation(t_echantillon), shapes_dict.(shape).yequation(t_echantillon), shapes_dict.(shape).zequation(t_echantillon)];
                assignin('base','datapoint', datapoint)
                spline(t,:)  = datapoint;
                assignin('base','spline', spline)
                targets(t,:) = datapoint;
                assignin('base','targets', targets)
        
                
                
                if t>1 
                    guesses = [j1(t-1,1),j2(t-1,1),j3(t-1,1),j4(t-1,1),j5(t-1,1)];
                    assignin('base','guesses', guesses)
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint,guesses);
                j1(t,1) = outputVec(1);
                j2(t,1) = outputVec(2);
                j3(t,1) = outputVec(3);
                j4(t,1) = outputVec(4);
                j5(t,1) = outputVec(5);
            end
            trajectory_motor_command{1}=j1;
            trajectory_motor_command{2}=j2;
            trajectory_motor_command{3}=j3;
            trajectory_motor_command{4}=j4;
            trajectory_motor_command{5}=j5;
            
            dataset_1=[dataset_1,five_motor_command_simulation(trajectory_motor_command,len_time_series,motorerrorselection,m1,m0,targets,model_name,mdl,base,follower,stationary_error,stationary_error_timestap,k,num_classes)];
        end
            dataset = [dataset,dataset_1];   
            fprintf("The final size of the dataset1 is %s", mat2str(size(dataset_1)));
            v1=mat2str(size(dataset_1));
    end 

    if secondtype
        dataset_2=[];
        shapes_dict=reduced_adapted_interpolate_set;
        shapelist=fieldnames(shapes_dict);
        numberofshapes=numel(shapelist);
        fprintf('The number of fields is:%d\n',numberofshapes);
        for k = 1:numberofshapes
            if k==floor(numberofshapes/4)
                disp("------------------------")
                disp("K =1/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes/2)
                disp("------------------------")
                disp("K =1/2 HAS BEEN REACHED")
                disp("------------------------")
            end
                if k==floor(numberofshapes*3/4)
                disp("------------------------")
                disp("K =3/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            
        
            shape=shapelist{k};
            disp("------------------------")
            fprintf('Shape name:%s\n',shape);
            disp("------------------------")
        
            %disp("Equations de la forme");
            %disp(shapes_dict.(shape));
            x=shapes_dict.(shape).xcoords;
            y=shapes_dict.(shape).ycoords;
            z=shapes_dict.(shape).zcoords;
        
        
            for t = 1:len_time_series
                %t_echantillon=t/500;
                datapoint =[x(t), y(t),z(t)];
                assignin('base','datapoint', datapoint)
                % datapoint =[shapes_dict.(shape).xequation(t_echantillon), shapes_dict.(shape).yequation(t_echantillon), shapes_dict.(shape).zequation(t_echantillon)];
                %datapoint = [0+k*0.1*cos(t/100*(2*pi/T)),0+k*0.1*sin(t/100*(2*pi/T)),0.15+k*0.1*(t/100/T)];
                spline(t,:)  = datapoint;
                assignin('base','spline', spline)
                targets(t,:) = datapoint;
                assignin('base','targets', targets)
        
                
                
                if t>1 
                    guesses = [j1(t-1,1),j2(t-1,1),j3(t-1,1),j4(t-1,1),j5(t-1,1)];
                    assignin('base','guesses', guesses)
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint,guesses);
                j1(t,1) = outputVec(1);
                j2(t,1) = outputVec(2);
                j3(t,1) = outputVec(3);
                j4(t,1) = outputVec(4);
                j5(t,1) = outputVec(5);
            end
          
    % %%%% new code for drawing command graph
    %      figure;
    %     plot3(x, y, z, 'LineWidth', 2);
    %     hold on;
    % 
    %     % Scatter plot with color gradient based on point index
    %     scatter3(x(1:10:end), y(1:10:end), z(1:10:end), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
    % 
    %     title(['Reference Trajectory ']);
    %     xlabel('X-axis');
    %     ylabel('Y-axis');
    %     zlabel('Z-axis');
    %     grid on;
    % 
    %     % Force MATLAB to update the figure window
    %     drawnow;
    % 
    % 
    % %%%%%
            trajectory_motor_command{1}=j1;
            trajectory_motor_command{2}=j2;
            trajectory_motor_command{3}=j3;
            trajectory_motor_command{4}=j4;
            trajectory_motor_command{5}=j5;
            
            dataset_2=[dataset_2,five_motor_command_simulation(trajectory_motor_command,len_time_series,motorerrorselection,m1,m0,targets,model_name,mdl,base,follower, stationary_error,stationary_error_timestap,k,num_classes)];
        end
        dataset = [dataset,dataset_2];   
        fprintf("The final size of the dataset2 is %s", mat2str(size(dataset_2)));
        v2=mat2str(size(dataset_2));
    end


    if thirdtype
        dataset_3=[];
        shapes_dict=realistic_trajs_set;
        numberofshapes=sum(cellfun(@iscell, shapes_dict));
        fprintf('The number of cells is:%d\n',numberofshapes);
        for k = 1:numberofshapes
            if k==floor(numberofshapes/4)
                disp("------------------------")
                disp("K =1/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes/2)
                disp("------------------------")
                disp("K =1/2 HAS BEEN REACHED")
                disp("------------------------")
            end
                if k==floor(numberofshapes*3/4)
                disp("------------------------")
                disp("K =3/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            
        
            shape=shapes_dict{k};
            j1=shape{1};
            j2=shape{2};
            j3=shape{3};
            j4=shape{4}; 
            j5=shape{5};
            [x,y,z]=ForwardKinematic(j1, j2, j3, j4, j5,len_time_series,mdl,base,follower);
            targets=[x,y,z];
            assignin('base','targets', targets)
            % target_dimensions= size(targets);
            % fprintf('The final size of targets is %d * %d .\n',target_dimensions(1),target_dimensions(2));
            spline  = targets;
            assignin('base','spline', spline)
            trajectory_motor_command=shape;
            dataset_3=[dataset_3,five_motor_command_simulation(trajectory_motor_command,len_time_series,motorerrorselection,m1,m0,targets,model_name,mdl,base,follower,stationary_error,stationary_error_timestap,k,num_classes)];
        end
        dataset = [dataset,dataset_3];   
        fprintf("The final size of the dataset2 is %s", mat2str(size(dataset_3)));
        v2=mat2str(size(dataset_3));
    end
    if simulating
        % Specify the size of each submatrix (6x1000)
        fprintf("The final size of the fulldataset is %s", mat2str(size(dataset)));
        v3=mat2str(size(dataset));
        dataset = dataset';
        %clear size
        sized = size(dataset);
    
        rowDist = 6 * ones(1, sized(1)/6);
        % Use mat2cell to convert the dataset into a cell array
        cellArray = mat2cell(dataset, rowDist);
        %disp(size(cellArray))
        
        save(trajectory_dataset_name, 'cellArray');
        disp('Dataset saved')
        % Now, cellArray is a cell array where each cell is a 6x1000 matrix

    end 

    %Running the rain_predict_file
    if training 
        disp('Running AI model')
        run(trained_AI_model);
        
    end
    %run('rain_predict_lstm.m');
    %run("optimum_train_predict.m")

    %%% end of experimental section %%%


end











%Support functions

%function0
function reducedStruct = reduceStructureSize(inputStruct, n)
    % Get field names of the input structure
    fieldNames = fieldnames(inputStruct);
    
    % Check if n is greater than the number of fields
    if n > numel(fieldNames)
        error('n is greater than the number of fields in the input structure.');
    end
    
    % Randomly select n field names
    selectedFields = datasample(fieldNames, n, 'Replace', false);
    
    % Create the reduced structure
    reducedStruct = struct();
    for i = 1:numel(selectedFields)
        fieldName = selectedFields{i};
        reducedStruct.(fieldName) = inputStruct.(fieldName);
    end
end

%function1
function R = calculateRotationMatrix(anglexy, anglexz, angleyz)
    % Conversion des angles en radians
    anglexy = deg2rad(anglexy);
    anglexz = deg2rad(anglexz);
    angleyz = deg2rad(angleyz);

    % Matrices de rotation autour des axes
    Rz = [cos(anglexy) -sin(anglexy) 0; sin(anglexy) cos(anglexy) 0; 0 0 1];
    Rx = [1 0 0; 0 cos(anglexz) -sin(anglexz); 0 sin(anglexz) cos(anglexz)];
    Ry = [cos(angleyz) 0 sin(angleyz); 0 1 0; -sin(angleyz) 0 cos(angleyz)];

    % Calcul de la matrice totale de rotation
    R = Rz * Rx * Ry;
end


%function2
function newCell = multiplyandsum(r,matrix1,Cell,matrix2)

newCell = cell(size(Cell));
   for i=1:numel(Cell)
      

       scaledfunction= @(x) r*(matrix1(i, 1) *Cell{1}(x) + matrix1(i, 2) * Cell{2}(x) + matrix1(i, 3) * Cell{3}(x))+matrix2(i);
       newCell{i}= scaledfunction;
   end
end

%function3
function structure3 = mergeStructures(structure1, structure2)
    % Copy the contents of structure1 to structure3
    structure3 = structure1;
    
    % Get field names of structure2
    fields2 = fieldnames(structure2);

    % Iterate through fields of structure2 and add them to structure3
    for i = 1:length(fields2)
        field = fields2{i};
        structure3.(field) = structure2.(field);
    end
end



%function4
function [circlelist] = CreateCircleList(max_rayon,max_eloignement_centre)
        %il va falloir limitter les paramètres maximum de la génération de
        %form en fonction du mouvement permi par le bras
        %pas de 0.01 choisi dans premières boucles for experimentalement
        %valeures de min et max eloignement aussi
        %Valeures charac du robot
        min_eloignement=0.02;
        max_eloignement=0.28;
        circlelist=struct();
        x_prime_z_prime_y_prime_coords={@(t) cos(2*pi*t);@(t) sin(2*pi*t);@(t) 0};
        for e_h=10:1:max_eloignement_centre*100                      %on itère sur les rayons possibles#changer incrémentation
            e=e_h*0.01;
            for r_h =1:1:max_rayon*100                             %on itère sur l'éloignement au centre possible #changer incrémentation?
                r=r_h*0.01;
                %condition d'appliquabilité à la simulation,trop forte
                %vire des points qui marche mais peu de calculs
                if (abs(e-r)<min_eloignement) || (e+r<min_eloignement) || (e+r>max_eloignement) || (abs(e-r)>max_eloignement)
                    continue
                end
                for anglexy=0:120:360                         %on explore les plans possibles en effectuant des rotations du plan xy autour de z
                    for anglexz=0:120:360                     %on explore les plans possibles en effectuant des rotations du plan xz autour de y
                        for angleyz=0:120:360                 %on explore les plans possibles en effectuant des rotations du plan yz autour de x          
                            for anglez=0:120:360              %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan z
                                for anglex=0:120:360          %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan x
                                    for angley=0:120:360      %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan y
                                        %coordonnées du centre du cercle
                                        %tracé
                                        ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
                                        Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
                                        circlecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);
                                        thiscircle=struct();
                                        circlename = sprintf('c_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
                                        fieldName = sprintf('xequation');
                                        thiscircle.(fieldName)=circlecoords{1};
                                        fieldName = sprintf('yequation');
                                        thiscircle.(fieldName)=circlecoords{2};
                                        fieldName = sprintf('zequation');
                                        thiscircle.(fieldName)=circlecoords{3};
                                        circlelist.(circlename) = thiscircle;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
end



%function5
function [linelist] = CreateLineList(max_eloignement_centre,max_longueur)
        %il va falloir limitter les paramètres maximum de la génération de
        %form en fonction du mouvement permi par le bras
        linelist=struct();
        min_eloignement=0.02;
        max_eloignement=0.28;
        x_prime_z_prime_y_prime_coords={@(t) (t<=max_longueur)*t;@(t) 0;@(t) 0};
        for e_h=10:1:max_eloignement_centre*100         %on itère sur l'éloignement au centre possible #changer incrémentation?
            e=e_h*0.01;
            for r_h =1:1:max_longueur*100
                r=r_h*0.01;
                %condition d'appliquabilité à la simulation, trop forte
                %vire des points qui marche mais peu de calculs
                if (abs(e-r)<min_eloignement) || (e+r<min_eloignement) || (e+r>max_eloignement) || (abs(e-r)>max_eloignement)
                    continue
                end
                for anglexy=0:120:360                       %on explore les plans possibles en effectuant des rotations du plan xy autour de z
                    for anglexz=0:120:360                   %on explore les plans possibles en effectuant des rotations du plan xz autour de y
                        for angleyz=0:120:0%fait rien ici 0 %on explore les plans possibles en effectuant des rotations du plan yz autour de x          
                            for anglez=0:120:360            %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan z
                                for anglex=0:120:360        %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan x
                                    for angley=0:120:360    %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan y
                                        %coordonnées du centre du cercle
                                        %tracé
                                        ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
                                        Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
                                        linecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);
                                        thisline=struct();
                                        linename = sprintf('l_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
                                        fieldName = sprintf('xequation');
                                        thisline.(fieldName)=linecoords{1};
                                        fieldName = sprintf('yequation');
                                        thisline.(fieldName)=linecoords{2};
                                        fieldName = sprintf('zequation');
                                        thisline.(fieldName)=linecoords{3};
                                        linelist.(linename) = thisline;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
end


%function6
% function [rectanglelist] = Create_rectangle_List(max_eloignement_centre,max_longueur,max_largeur)
%         %il va falloir limitter les paramètres maximum de la génération de
%         %form en fonction du mouvement permi par le bras
%         rectanglelist=struct();
%         r=1;
%         x_prime_z_prime_y_prime_coords={@(t) (t<=max_longueur)t-(max_longueur+max_largeur<t<=2*max_longueur+max_largeur)(t-max_longueur+max_largeur);@(t) (max_longueur<t<=max_longueur+max_largeur)(t-max_longueur) - (2*max_longueur+max_largeur<t<=2*max_longueur+2*max_largeur)(t-2*max_longueur+max_largeur);@(t) 0};
%         for e=0:max_eloignement_centre                  %on itère sur l'éloignement au centre possible #changer incrémentation?
%             for anglexy=0:120:360                       %on explore les plans possibles en effectuant des rotations du plan xy autour de z
%                 for anglexz=0:120:360                   %on explore les plans possibles en effectuant des rotations du plan xz autour de y
%                     for angleyz=0:120:360               %on explore les plans possibles en effectuant des rotations du plan yz autour de x          
%                         for anglez=0:120:360            %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan z
%                             for anglex=0:120:360        %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan x
%                                 for angley=0:120:360    %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan y
%                                     %coordonnées du centre du cercle
%                                     %tracé
%                                     ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
%                                     Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
%                                     rectanglecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);
%                                     thisrectangle=struct();
%                                     rectanglename = sprintf('r_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r, e, anglexy, anglexz, angleyz, anglez, anglex, angley);
%                                     fieldName = sprintf('xequation');
%                                     thisrectangle.(fieldName)=rectanglecoords{1};
%                                     fieldName = sprintf('yequation');
%                                     thisrectangle.(fieldName)=rectanglecoords{2};
%                                     fieldName = sprintf('zequation');
%                                     thisrectangle.(fieldName)=rectanglecoords{3};
%                                     rectanglelist.(rectanglename) = thisrectangle;
%                                 end
%                             end
%                         end
%                     end
%                 end
%             end
%         end
% end

%function7
function [interpolated_set] = createInterpolate(numberofinterpolatedshapes,len_time_series)
    %Interpolation set creation
    interpolated_set = struct();
    min_eloignement_point=0.02;
    max_eloignement_point=0.28;
    
    for p = 1:numberofinterpolatedshapes
        thisshape=struct();
        num_point = randi([3, 10]); % number of point for interpolation
        m = (max_eloignement_point - min_eloignement_point) * rand(3, num_point) + min_eloignement_point;
    
        %verification of sufficient Z value
        for i = 1:num_point
            if m(3,i)<0.1
                m(3,i)=0.1+(max_eloignement_point-0.1)*m(3,i);
            end
        end
    
        shapename = sprintf('ishape_p%d_num_point%d', p, num_point);
        X = m(1,:);Y = m(2,:);Z = m(3,:);
        values = spcrv([X(1) X X(end);Y(1) Y Y(end);Z(1) Z Z(end)],4);
        % plot3(X,Y,Z)
        % 
        % plot3(values(1,:),values(2,:),values(3,:))
        
        ts_x = timeseries(values(1,:),linspace(0,10,size(values,2)));
        ts_y = timeseries(values(2,:),linspace(0,10,size(values,2)));
        ts_z = timeseries(values(3,:),linspace(0,10,size(values,2)));
        
        end_time_value_in_seconds= (len_time_series-1)*0.01;

        ts_x = resample(ts_x, 0:0.01:end_time_value_in_seconds);
        ts_y = resample(ts_y, 0:0.01:end_time_value_in_seconds);
        ts_z = resample(ts_z, 0:0.01:end_time_value_in_seconds);

        % ts_x = resample(ts_x, 0.01:0.01:10);
        % ts_y = resample(ts_y, 0.01:0.01:10);
        % ts_z = resample(ts_z, 0.01:0.01:10);
        % 
        
        x = ts_x.Data(:);
        y = ts_y.Data(:);
        z = ts_z.Data(:);
        fieldName = sprintf('xcoords');
        thisshape.(fieldName)=x;
        fieldName = sprintf('ycoords');
        thisshape.(fieldName)=y;
        fieldName = sprintf('zcoords');
        thisshape.(fieldName)=z;
        interpolated_set.(shapename) = thisshape;
    end
end

%function8
function [circlelist] = CreateRandomCircleList(max_rayon, num_trajectories)
    max_eloignement_centre=max_rayon;
    min_eloignement = 0.02;
    max_eloignement = 0.28;
    min_hauteur = 0.1;
    circlelist = struct();
    x_prime_z_prime_y_prime_coords = {@(t) cos(2*pi*t); @(t) sin(2*pi*t); @(t) 0};

    % Initialize counter
    generated_trajectories = 0;

    % Keep generating trajectories until the desired number is reached
    while generated_trajectories < num_trajectories
        % Choose indices randomly
        e_values = generateRandomNumbers(min_hauteur*100, max_eloignement_centre*100, 1);
        r_values = generateRandomNumbers(1, max_rayon*100, 1);
        anglexy_values = generateRandomNumbers(1, 360, 1);
        anglexz_values = generateRandomNumbers(1, 360, 1);
        angleyz_values = generateRandomNumbers(1, 360, 1);
        anglez_values = generateRandomNumbers(1, 360, 1);
        anglex_values = generateRandomNumbers(1, 360, 1);
        angley_values = generateRandomNumbers(1, 360, 1);

        % Create a single trajectory
        for i = 1:1
            % Choosing values
            e_h = e_values(i);
            r_h = r_values(i);
            e = e_h * 0.01;
            r = r_h * 0.01;

            % Filtering values that would cause clipping
            if (abs(e-r) < min_eloignement) || (e+r < min_eloignement) || (e+r > max_eloignement) || (abs(e-r) > max_eloignement)
                continue
            end

            anglexy = anglexy_values(i);
            anglexz = anglexz_values(i);
            angleyz = angleyz_values(i);
            anglez = anglez_values(i);
            anglex = anglex_values(i);
            angley = angley_values(i);

            % Writing the functions
            ecoord = e * calculateRotationMatrix(anglez, angley, anglex) * [1; 1; 1];
            Rotation = calculateRotationMatrix(anglexy, anglexz, angleyz);
            circlecoords = multiplyandsum(r, Rotation, x_prime_z_prime_y_prime_coords, ecoord);

            % Create a structure for this trajectory
            thiscircle = struct();
            circlename = sprintf('c_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
            fieldName = sprintf('xequation');
            thiscircle.(fieldName) = circlecoords{1};
            fieldName = sprintf('yequation');
            thiscircle.(fieldName) = circlecoords{2};
            fieldName = sprintf('zequation');
            thiscircle.(fieldName) = circlecoords{3};

            % Add this trajectory to the structure
            circlelist.(circlename) = thiscircle;

            % Increment the counter
            generated_trajectories = generated_trajectories + 1;
        end
    end
end

%function9
function [linelist] = CreateRandomLineList(max_longueur, num_trajectories)
    max_eloignement_centre =max_longueur;
    min_eloignement = 0.02;
    max_eloignement = 0.28;
    min_hauteur = 0.1;
    linelist=struct();
    x_prime_z_prime_y_prime_coords={@(t) (t<=max_longueur)*t;@(t) 0;@(t) 0};
    % Initialize counter
    generated_trajectories = 0;

    % Keep generating trajectories until the desired number is reached
    while generated_trajectories < num_trajectories
        % Choose indices randomly
        e_values = generateRandomNumbers(min_hauteur*100, max_eloignement_centre*100, 1);
        r_values = generateRandomNumbers(1, max_longueur*100, 1);
        anglexy_values = generateRandomNumbers(1, 360, 1);
        anglexz_values = generateRandomNumbers(1, 360, 1);
        angleyz_values = generateRandomNumbers(1, 360, 1);
        anglez_values = generateRandomNumbers(1, 360, 1);
        anglex_values = generateRandomNumbers(1, 360, 1);
        angley_values = generateRandomNumbers(1, 360, 1);

        % Create a single trajectory
        for i = 1:1
            % Choosing values
            e_h = e_values(i);
            r_h = r_values(i);
            e = e_h * 0.01;
            r = r_h * 0.01;

            % Filtering values that would cause clipping
            if (abs(e-r) < min_eloignement) || (e+r < min_eloignement) || (e+r > max_eloignement) || (abs(e-r) > max_eloignement)
                continue
            end

            anglexy = anglexy_values(i);
            anglexz = anglexz_values(i);
            angleyz = angleyz_values(i);
            anglez = anglez_values(i);
            anglex = anglex_values(i);
            angley = angley_values(i);

            % Writing the functions
            ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
            Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
            linecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);

            % Create a structure for this trajectory
            thisline=struct();
            linename = sprintf('l_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
            fieldName = sprintf('xequation');
            thisline.(fieldName)=linecoords{1};
            fieldName = sprintf('yequation');
            thisline.(fieldName)=linecoords{2};
            fieldName = sprintf('zequation');
            thisline.(fieldName)=linecoords{3};

            % Add this trajectory to the structure
            linelist.(linename) = thisline;

            % Increment the counter
            generated_trajectories = generated_trajectories + 1;
        end
    end
end

%function10
function randomIntegers = generateRandomNumbers(a, b, p)
    % Generate p random integers in the interval [a, b]
    randomIntegers = randi([round(a), round(b)], 1, p);
end





%function 11
function [output_dataset] = five_motor_command_simulation(input_motor_commands,len_time_series,motorerrorselection,m1,m0,targets,model_name,mdl,base,follower,stationary_error,stationary_error_timestap,k,num_classes)
%_______________
%Creates a matrix that contains the wanted trajectory dataset with required
%errors based on the inputed trajectory types give inside of the motor
%commands (therfore there are (number of trajectories)*(number of errors)
%trajectories in the output dataset).
%
%   input_motor_commands:cell containing motor command dataset ( for each trajetory: 5 motor commands, 1 for
%   each motor)
%_______________
        output_dataset=[];
        
        j1 = 180 + input_motor_commands{1};
        j2 = -input_motor_commands{2};
        j3 = input_motor_commands{3};
        j4 = -input_motor_commands{4};
        j5 = input_motor_commands{5};
        
        end_time_value_in_seconds= (len_time_series-1)*0.01;
    
        joint1_ts = timeseries(j1/180*pi,0:0.01:end_time_value_in_seconds);
        assignin('base','joint1_ts', joint1_ts)
        joint2_ts = timeseries(j2/180*pi,0:0.01:end_time_value_in_seconds);
        assignin('base','joint2_ts', joint2_ts)
        joint3_ts = timeseries(j3/180*pi,0:0.01:end_time_value_in_seconds);
        assignin('base','joint3_ts', joint3_ts)
        joint4_ts = timeseries(j4/180*pi,0:0.01:end_time_value_in_seconds);
        assignin('base','joint4_ts', joint4_ts)
        joint5_ts = timeseries(j5/180*pi,0:0.01:end_time_value_in_seconds);
        assignin('base','joint5_ts', joint5_ts)

        %Creating reset values to but things back to normal after error has
        %been simulated
       % Initialize placeholders for original data
placeholder1 = joint1_ts.Data;
placeholder2 = joint2_ts.Data;
placeholder3 = joint3_ts.Data;
placeholder4 = joint4_ts.Data;
placeholder5 = joint5_ts.Data;

for j = motorerrorselection
    disp('----------')
    fprintf('Motor off is: %d\n', j);
    fprintf('Progression is: %d\n', k);
    disp('----------')

    % Reset errors to initial values
    error1 = m1;
    error2 = m1;
    error3 = m1;
    error4 = m1;
    error5 = m1;
    error6 = m1;
    
    % Assign base workspace variables
    assignin('base', 'error1', error1)
    assignin('base', 'error2', error2)
    assignin('base', 'error3', error3)
    assignin('base', 'error4', error4)
    assignin('base', 'error5', error5)
    assignin('base', 'error6', error6)
    
    % Reset joint data to original placeholders
    joint1_ts.Data = placeholder1;
    joint2_ts.Data = placeholder2;
    joint3_ts.Data = placeholder3;
    joint4_ts.Data = placeholder4;
    joint5_ts.Data = placeholder5;
    initial_data = {joint1_ts.Data, joint2_ts.Data, joint3_ts.Data, joint4_ts.Data, joint5_ts.Data};

   assignin('base', 'joint1_ts', setfield(joint1_ts, 'Data', placeholder1));
    assignin('base', 'joint2_ts', setfield(joint2_ts, 'Data', placeholder2));
    assignin('base', 'joint3_ts', setfield(joint3_ts, 'Data', placeholder3));
    assignin('base', 'joint4_ts', setfield(joint4_ts, 'Data', placeholder4));
    assignin('base', 'joint5_ts', setfield(joint5_ts, 'Data', placeholder5));

    % Apply errors or process points based on the current motor selection
    switch j
        case 0
            error1 = m1;
            assignin('base', 'error1', error1)
        case 1
            error1 = m0;
            assignin('base', 'error1', error1)
        case 2
            error2 = m0;
            assignin('base', 'error2', error2)
        case 3  
            error3 = m0;
            assignin('base', 'error3', error3)
        case 4  
            error4 = m0;
            assignin('base', 'error4', error4)
        case 5  
            error5 = m0;
            assignin('base', 'error5', error5)
        case 6
            joint1_ts.Data = process_points(joint1_ts.Data);
            assignin('base', 'joint1_ts', joint1_ts)
        case 7
            joint2_ts.Data = process_points(joint2_ts.Data);
            assignin('base', 'joint2_ts', joint2_ts)
        case 8
            joint3_ts.Data = process_points(joint3_ts.Data);
            assignin('base', 'joint3_ts', joint3_ts)
        case 9
            joint4_ts.Data = process_points(joint4_ts.Data);
            assignin('base', 'joint4_ts', joint4_ts)
        case 10
            joint5_ts.Data = process_points(joint5_ts.Data);
            assignin('base', 'joint5_ts', joint5_ts)
    
                case 11
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    
                    joint1_ts.Data = extend_trajectory(joint1_ts.Data, rand());
                    assignin('base','joint1_ts', joint1_ts)
                case 12
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;

                    joint2_ts.Data = extend_trajectory(joint2_ts.Data, rand());
                    assignin('base','joint2_ts', joint2_ts)
                case 13
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;

                    joint3_ts.Data = extend_trajectory(joint3_ts.Data, rand());
                    assignin('base','joint3_ts', joint3_ts)
                case 14
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;

                    joint4_ts.Data = extend_trajectory(joint4_ts.Data, rand());
                    assignin('base','joint4_ts', joint4_ts)
                case 15
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;

                    joint5_ts.Data = extend_trajectory(joint5_ts.Data, rand());
                    assignin('base','joint5_ts', joint5_ts)
                case 16
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint1_ts.Data = process_points_capped_speed(joint1_ts.Data, speedcap, timescale);
                    assignin('base','joint1_ts', joint1_ts)
                case 17
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint2_ts.Data = process_points_capped_speed(joint2_ts.Data, speedcap, timescale);
                    assignin('base','joint2_ts', joint2_ts)
                case 18
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint3_ts.Data = process_points_capped_speed(joint3_ts.Data, speedcap, timescale);
                    assignin('base','joint3_ts', joint3_ts)
                case 19
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint4_ts.Data = process_points_capped_speed(joint4_ts.Data, speedcap, timescale);
                    assignin('base','joint4_ts', joint4_ts)
                case 20
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint5_ts.Data = process_points_capped_speed(joint5_ts.Data, speedcap, timescale);
                    assignin('base','joint5_ts', joint5_ts)
                case 21
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint1_ts.Data = process_points_stationary_error(joint1_ts.Data, stationary_error, stationary_error_timestap);
                    assignin('base','joint1_ts', joint1_ts)
                    
                case 22
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint2_ts.Data = process_points_stationary_error(joint2_ts.Data, stationary_error, stationary_error_timestap);
                    assignin('base','joint2_ts', joint2_ts)
                case 23
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                
                    joint3_ts.Data = process_points_stationary_error(joint3_ts.Data, stationary_error, stationary_error_timestap);
                    assignin('base','joint3_ts', joint3_ts)
                case 24
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                    % 
                    joint4_ts.Data = process_points_stationary_error(joint4_ts.Data, stationary_error, stationary_error_timestap);
                    assignin('base','joint4_ts', joint4_ts)
                case 25
                    % joint1_ts.Data = placeholder1;
                    % joint2_ts.Data = placeholder2;
                    % joint3_ts.Data = placeholder3;
                    % joint4_ts.Data = placeholder4;
                    % joint5_ts.Data = placeholder5;
                
                    joint5_ts.Data = process_points_stationary_error(joint5_ts.Data, stationary_error, stationary_error_timestap);
                    assignin('base','joint5_ts', joint5_ts)
            end
            output_dataset = [output_dataset, targets];
    
            %on ajoute déjà les trajectoires cibles
            disp("----------------")
            disp("----------------")
            w=warning('off','all');
            simOut = sim(model_name);
            warning(w);
            assignin('base','simOut', simOut)
            disp("----------------")
            disp("----------------")
    
            j1o = simOut.j1.Data;
            j2o = simOut.j2.Data;
            j3o = simOut.j3.Data;
            j4o = simOut.j4.Data;
            j5o = simOut.j5.Data;
            j1o = j1o*180/pi;
            j2o = j2o*180/pi;
            j3o = j3o*180/pi;
            j4o = j4o*180/pi;
            j5o = j5o*180/pi;

    joint1_ts.Data = placeholder1;
    joint2_ts.Data = placeholder2;
    joint3_ts.Data = placeholder3;
    joint4_ts.Data = placeholder4;
    joint5_ts.Data = placeholder5;
    disp(placeholder1)
figure;

% Plot for joint 1
subplot(5, 1, 1); % 5 rows, 1 column, 1st plot
plot(joint1_ts.Time, joint1_ts.Data);
title('Joint 1');
xlabel('Time');
ylabel('Value');
grid on;

% Plot for joint 2
subplot(5, 1, 2); % 5 rows, 1 column, 2nd plot
plot(joint2_ts.Time, joint2_ts.Data);
title('Joint 2');
xlabel('Time');
ylabel('Value');
grid on;

% Plot for joint 3
subplot(5, 1, 3); % 5 rows, 1 column, 3rd plot
plot(joint3_ts.Time, joint3_ts.Data);
title('Joint 3');
xlabel('Time');
ylabel('Value');
grid on;

% Plot for joint 4
subplot(5, 1, 4); % 5 rows, 1 column, 4th plot
plot(joint4_ts.Time, joint4_ts.Data);
title('Joint 4');
xlabel('Time');
ylabel('Value');
grid on;

% Plot for joint 5
subplot(5, 1, 5); % 5 rows, 1 column, 5th plot
plot(joint5_ts.Time, joint5_ts.Data);
title('Joint 5');
xlabel('Time');
ylabel('Value');
grid on;
         
simulated_data = {simOut.j1.Data, simOut.j2.Data, simOut.j3.Data, simOut.j4.Data, simOut.j5.Data};
time = joint1_ts.Time; % Assuming all timeseries objects have the same time vector

%Plot the trajectories before and after simulation
for i = 2:2
    % Reshape data to be a column vector
    initial_data_i = reshape(initial_data{i}, [], 1);
    simulated_data_i = reshape(simulated_data{i}, [], 1);

    % Ensure time is a column vector
    time = time(:);

    % Create figure
    figure;
    hold on;

    % Plot initial data
    plot(time, initial_data_i, 'b-', 'DisplayName', 'Before Simulation');

    % Plot simulated data
    plot(time, simulated_data_i, 'r--', 'DisplayName', 'After Simulation');

    % Labels and title
    xlabel('Time');
    ylabel(['Joint ', num2str(i), ' Position']);
    title(['Joint ', num2str(i), ' Trajectories']);

    % Show legend
    legend('show');

    hold off;
end
                    
            [x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o, j5o,len_time_series,mdl,base,follower); 
            figure;
            % plot3(x_scaled, y_scaled, z, 'o-');
            plot3(x, y, z, 'o-');
            grid on;
            % Scatter plot with color gradient based on point index
            scatter3(x(1:10:end), y(1:10:end), z(1:10:end), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
            xlabel('X-axis');
            ylabel('Y-axis');
            zlabel('Z-axis');
            title('3D Plot Example, what should be done');


             jdatapoint = [x, y, z];%pour un j donné on met à la suite les len_time_series prédit  et les réels en prenant en compte le défault moteur, c'est ce qu'on donnera à manger à l'IA;
             output_dataset=[output_dataset,jdatapoint];
            
        end 
end

%function 12
function [x, y ,z] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series,mdl,base,follower)
    %joint1_damping = 0;
    %joint2_damping = 0;
    %damp_pince = 1000; % damping coefficient for joints of the pince
    
    %mdl = "robot_model";
    
    load_system(mdl)
    
    ik = simscape.multibody.KinematicsSolver(mdl);
    
    %base = "robot_model/World/W";
    %follower = "robot_model/gripper_base/F";
    addFrameVariables(ik,"gripper_base","translation",base,follower);
    addFrameVariables(ik,"gripper_base","rotation",base,follower);
    
    targetIDs = ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"] ;
    addTargetVariables(ik,targetIDs);
    outputIDs =["gripper_base.Translation.x";"gripper_base.Translation.y";...
        "gripper_base.Translation.z"];
    addOutputVariables(ik,outputIDs);
    
    x = zeros(len_time_series,1);
    y = zeros(len_time_series,1);
    z = zeros(len_time_series,1);
    T = 10; % period
    %spline = zeros(len_time_series,5);
   
    for i = 1:len_time_series
        targets = [j1(i),j2(i),j3(i),j4(i),j5(i)];
    

    
        [outputVec,statusFlag] = solve(ik,targets);
        x(i,1) = outputVec(1);
        y(i,1) = outputVec(2);
        z(i,1) = outputVec(3);
    
        
    end
    % fprintf('size_x is %d.\n',size(x));
    % fprintf('size_targets is %d.\n',size([x, y ,z] ));
    



end

function updated_j1 = process_points(j1)
    % Initialize pointsList
    pointsList = zeros(length(j1), 1);

    % Track if at least one block is set to zero
    zeroBlockExists = false;

    % Loop through each block of 200 points
    for i = 1:floor(length(j1)/200)
        start_index = (i-1)*200 + 1;
        end_index = i*200;

        % Generate a random number to decide if the block will be zeros or ones
        if rand <= 0.9
            pointsList(start_index:end_index) = 0;
            zeroBlockExists = true; % Set flag to true indicating at least one block is set to zero
        else
            pointsList(start_index:end_index) = 1;
        end
    end

    % If no block is set to zero, randomly select one block and set it to zero
    if ~zeroBlockExists
        blockIndex = randi(floor(length(j1)/200));
        start_index = (blockIndex-1)*200 + 1;
        end_index = blockIndex*200;
        pointsList(start_index:end_index) = 0;
    end

    % Track the indices of the last 50 zeros
    lastZerosIndices = find(pointsList == 0, 50, 'last');

    % Check if the next set in pointsList is a set of ones
    if length(pointsList) > lastZerosIndices(end) + 200 && all(pointsList(lastZerosIndices(end) + 1:lastZerosIndices(end) + 200) == 1)
        %disp("end of stoppage - interpolating to avoid jump")
        % Interpolate from the current value of j1 to the next one
        startValue = j1(lastZerosIndices(end));
        endValue = j1(lastZerosIndices(end) + 200);
        interpolatedValues = linspace(startValue, endValue, 200);
        j1(lastZerosIndices(end) + 1:lastZerosIndices(end) + 200) = interpolatedValues;
    end

    % Iterate over pointsList to set j1(i) to j1(i-1) where necessary
    %disp(pointsList)


    %pointsList = [ones(200,1);zeros(800, 1)];



    for i = 2:numel(pointsList)
        if pointsList(i) == 0
            j1(i) = j1(i-1);
        end
        
    end

    % Return the updated j1 list
    updated_j1 = j1;
end


%function 14
function updated_j1 = process_points_capped_speed(j1, cap, time_scale)
%sample values : cap = 2, time_scale  = 0.01
    for i = 2:numel(j1)
          
        if (j1(i)-j1(i-1))/time_scale > cap
            % Set j1(i) to j1(i-1)
            
            j1(i) = j1(i-1)+cap*time_scale;
        elseif (j1(i)-j1(i-1))/time_scale < - cap

            j1(i) = j1(i-1)-cap*time_scale;
            
        end
    end

    % Return the updated j1 list
    updated_j1 = j1;
end


function updated_j1 = process_points_stationary_error(j1, stationary_error, stationary_error_timestamp)
    % Define the length of each block
    blockSize = 200;
    
    % Number of blocks
    numBlocks = length(j1) / blockSize;
    
    % Generate a list of blocks to be updated
    blocksToUpdate = rand(numBlocks, 1) <= 0.5;
    
    % Ensure that at least one block will be updated
    if ~any(blocksToUpdate)
        blocksToUpdate(randi(numBlocks)) = true;
    end
    
    % Iterate through the blocks to be updated and apply the error
    for i = 1:numBlocks
        if blocksToUpdate(i)
            start_index = (i-1)*blockSize + 1;
            end_index = i*blockSize;
            j1(start_index:end_index) = j1(start_index:end_index) + stationary_error;
        end
    end
    
    % Return the updated j1
    updated_j1 = j1;
end


%function 15
% function updated_trajectory = extend_trajectory(originalPoints, scaleFactor)
%     % Determine the number of points in the original trajectory
%     numOriginalPoints = size(originalPoints, 1);
% 
%     % Create an index for the original points
%     originalIndices = 1:numOriginalPoints;
% 
%     % Create an extended index based on the scale factor
%     extendedIndices = linspace(1, numOriginalPoints, round(scaleFactor * numOriginalPoints));
% 
%     % Interpolate to get extended trajectory
% 
%     updated_trajectory = interp1(originalIndices, originalPoints, extendedIndices, 'linear', 'extrap');
% 
%     % Limit the extended trajectory to the original number of points
%     updated_trajectory = updated_trajectory(1:min(numOriginalPoints, length(extendedIndices)));
% end




function updated_trajectory = extend_trajectory(originalPoints, scaleFactor)
    % Initialize the updated trajectory with the original points
    updated_trajectory = originalPoints;
    
    % Define the length of each block
    blockSize = 200;
    
    % Number of blocks
    numBlocks = floor(length(originalPoints) / blockSize);
    
    % Track if at least one block has been resampled
    resampleMade = false;
    
    % Iterate through each block of 200 points
    for i = 1:numBlocks
        start_index = (i-1)*blockSize + 1;
        end_index = i*blockSize;
        
        % Generate a random number to decide if the first 100 points will be resampled
        if rand <= 0.5
            % Original first 100 points
            first_half = originalPoints(start_index:start_index+99);
            second_half = originalPoints(start_index+100:start_index+199);
            
            % Extend the first 100 points
            extended_points = extend_points(first_half, scaleFactor);
            
            % Downscale the second 100 points
            compressed_points = extend_points(second_half, scaleFactor);
            
            % Combine the extended and compressed points
            resampled_points = [extended_points, compressed_points];
            
            % Ensure resampled_points length matches the original block size
            if length(resampled_points) > blockSize
                resampled_points = resampled_points(1:blockSize);
            elseif length(resampled_points) < blockSize
                resampled_points = [resampled_points, zeros(1, blockSize - length(resampled_points))];
            end
            
            % Update the trajectory with resampled points
            updated_trajectory(start_index:end_index) = resampled_points;
            
            resampleMade = true; % Set flag to true indicating at least one block has been resampled
        end
    end
    
    % If no block has been resampled, randomly select one block to resample
    if ~resampleMade
        blockIndex = randi(numBlocks);
        start_index = (blockIndex-1)*blockSize + 1;
        end_index = blockIndex*blockSize;
        
        % Original first 100 points
        first_half = originalPoints(start_index:start_index+99);
        second_half = originalPoints(start_index+100:start_index+199);
        
        % Extend the first 100 points
        extended_points = extend_points(first_half, scaleFactor);
        
        % Downscale the second 100 points
        compressed_points = extend_points(second_half, scaleFactor);
        
        % Combine the extended and compressed points
        resampled_points = [extended_points, compressed_points];
        
        % Ensure resampled_points length matches the original block size
        if length(resampled_points) > blockSize
            resampled_points = resampled_points(1:blockSize);
        elseif length(resampled_points) < blockSize
            resampled_points = [resampled_points, zeros(1, blockSize - length(resampled_points))];
        end
        
        % Update the trajectory with resampled points
        updated_trajectory(start_index:end_index) = resampled_points;
    end
end

function extended_points = extend_points(points, scaleFactor)
    % Number of original points

    num_original_points = numel(points);
    % Number of points after extending
    num_extended_points = round(num_original_points * (1 + scaleFactor));
    
   % Original number of points
    
    
    % Reshape points to ensure it is a row vector
    points = reshape(points, 1, num_original_points);
    
    % New list of points
    new_points = linspace(0, 1, num_extended_points);
    

 
    % Linear interpolation to extend the original points
    extended_points = interp1(linspace(0, 1, num_original_points), points, new_points);

end


%function 16
function [trajectories,csv_file_equivalent] = createRandomPickupList(number_of_pickup_trajctories,len_time_series, zero_amount,average_smallest_motive_lenght)
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
%Interpolation set creation
    pickup_set = struct();
    min_eloignement_point=0.02;
    max_eloignement_point=0.28;
    
    max_number_of_motives=len_time_series/average_smallest_motive_lenght;

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
        allowed_amplitude = {80,80,80,80,80};
        danger_zone = false;
        for i = 1:5
            if danger_zone == true
                allowed_amplitude = {80,80,80,80,80};
            end
            if i ==2
            [motor_command,triplets] = realisticsinglemotorcommand(max_number_of_motives,len_time_series,zero_amount, allowed_amplitude{i});
            else
            [motor_command,triplets] = realisticsinglemotorcommandNOGENERATION(max_number_of_motives,len_time_series,zero_amount, allowed_amplitude{i});    
            end
            if i ==1
                
                if -50<triplets(1,1)<50
                    danger_zone = true;
                end
            end
            trajectory{i} = motor_command;
            triplets_cell{i} = triplets;
        end

        trajectories{generated_trajectories} = trajectory;
        csv_file_equivalent{generated_trajectories} = triplets_cell;
       
    end
end


%function 17
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

speed_cap = 2.7;

% Random number of motives on the command of each motor
% Note to self: maybe favorise appearance of 0 more often with better mechanism?

% Initialising the points at 0
motor_command = zeros(len_time_series, 1);

% Choice of motor use or not
percentage_zero_amount = zero_amount * 100;
toggle_value = generateRandomNumbers(1, 100, 1);
average_smallest_motive_length = len_time_series / max_number_of_motives;

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
        arrival_to_plateau_proportion = randi([2, 8]) / 10;
    
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
        
        if rand() < 0.6
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
function [motor_command, triplets] = realisticsinglemotorcommandNOGENERATION(max_number_of_motives, len_time_series, zero_amount, allowed_amplitude)
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

speed_cap = 2.7;

% Random number of motives on the command of each motor
% Note to self: maybe favorise appearance of 0 more often with better mechanism?

% Initialising the points at 0
motor_command = zeros(len_time_series, 1);

% Choice of motor use or not
percentage_zero_amount = zero_amount * 100;
toggle_value = generateRandomNumbers(1, 100, 1);
average_smallest_motive_length = len_time_series / max_number_of_motives;

% Application of the toggle
    triplets = [0, 0, 0];
    motor_command(1) = 0.0001;
    motor_command(1) = 0.0002;

end   