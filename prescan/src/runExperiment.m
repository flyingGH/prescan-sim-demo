function [] = runExperiment(target, host_ip)

% This function reads the trajectory of a specific object in a Prescan experiment
% and publishes it as a ROS message of type nav_msgs/Path
rosshutdown;
 
% Target PC
rosinit(target);
setenv('ROS_MASTER_URI', ['http://' target ':11311'])

% This PC, mandatory
setenv('ROS_IP', host_ip)
setenv('ROS_HOSTNAME', host_ip)

% --------------------------------------------------------------------------------------------------------

% vecSize is the number of waypoints in the path array
vecSize = 500; % Default value

% load vehicle configuration file
vec_info = readstruct("vehicle_configuration.xml");
vec_names = fieldnames(vec_info);
vec_physical_names = {};
for i = 1:length(vec_names)
    if ~str2num(vec_info.(vec_names{i}).virtual)
        vec_physical_names{end+1} = vec_names{i};
    end
end


% --------------------------------------------------------------------------------------------------------
%This conmmand is equvilant of pressing "Build" from the GUI
%It writes the content of the GUI into the PB file
%So in refreshes your PB file and overwrite any previously made changes
%This is useful when multiple experiments are run one after the other and
%prevents che changes from previous run co1 still be present in chis
%current run

% Convert the Prescan experiment to data models
prescan.api.experiment.loadExperimentFromFile('PrescanDemoAMC.pb');  % Convert the Prescan experiment files into MATLAB data models

% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file
% --------------------------------------------------------------------------------------------------------
SimulinkFileName = pbFileName(1:end-3)+"_cs";

open_system(SimulinkFileName);

% Change trigger constant for physical/virtual vehicles
for i = 1:length(vec_names)
    if str2num(vec_info.(vec_names{i}).virtual)
        assignin('base', sprintf('TriggerCar_%d', i), 0);
    else
        assignin('base', sprintf('TriggerCar_%d', i), 1);
    end
end

% --------------------------------------------------------------------------------------------------------
Agents_ID

% --------------------------------------------------------------------------------------------------------

% Create and publish a path message for each car
for idx = 1:length(vec_physical_names)
    objectName = vec_physical_names{idx}; % Get the name of the current car
    %pathTopic = ['car_', num2str(idx), '/path']; % Create a unique topic for this car
    % Following line just for nowtt
    physical_id = vec_info.(vec_physical_names{idx}).physical_id;
    pathTopic = ['car_', num2str(physical_id), '/path']; % Create a unique topic for this car

    % Create a ROS publisher for the car's path
    pathPub = rospublisher(pathTopic, 'nav_msgs/Path', 'IsLatching', true, 'DataFormat', 'struct');
    pathmsg = rosmessage(pathPub);
    
    % Set up path message header
    pathmsg.Header.FrameId = 'f1Tenth_base';
    pathmsg.Header.Stamp = rostime('now','DataFormat','struct');
    
    % Initialize pose array
    poseArray = repmat(rosmessage('geometry_msgs/PoseStamped', 'DataFormat', 'struct'), vecSize, 1);
    
    % Get the object and its trajectory
    carObject = experiment.getObjectByName(objectName);
    trajectory = prescan.api.trajectory.getAttachedTrajectories(carObject);
    path = trajectory.path;
    
    % Calculate positions and orientations for each waypoint
    factor = path.length/vecSize;
    for i = 1:vecSize
        pose = path.poseAtDistance(factor*i);
        poseArray(i).Header.FrameId = 'f1Tenth_base';
        poseArray(i).Header.Stamp = rostime('now', 'DataFormat', 'struct');
        poseArray(i).Pose.Position.X = pose.position.x/10;
        poseArray(i).Pose.Position.Y = pose.position.y/10;
        poseArray(i).Pose.Position.Z = pose.position.z;
        Q = angle2quat(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw, 'XYZ');
        poseArray(i).Pose.Orientation = struct('X', Q(1), 'Y', Q(2), 'Z', Q(3), 'W', Q(4));
    end
    
    % Assign the pose array to the path message and publish it
    pathmsg.Poses = poseArray;
    send(pathPub, pathmsg);
    
    % Print a message indicating that the path is being published
    disp(['Publishing path for ' objectName]);
    clear('pub','node');
end

%%

% Run the Prescan experiment
prescan.api.simulink.run(experiment, 'Regenerate', 'off');