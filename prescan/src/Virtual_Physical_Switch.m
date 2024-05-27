% ==========================================================================================================
% this code can be used to create a switch block so that the position of
% the agent in Prescan can be obtained by either the Prescan path or the
% ROS message from the Vicon system
clc;
clearvars -except NUTS_defined

% ========================================= Scaling of the MAP =============================================
MapScaling = 10;
% ==========================================================================================================
% --------------------------------------------------------------------------------------------------------
% This conmmand is equvilance of pressing "Build" from the GUI
% It writes the content of the GUI into the PB file
% So in refreshes your PB file and overwrite any previously made changes
% This is useful when multiple experiments are run one after the other and
% prevents changes from previous run still be present in current run

% Convert the Prescan experiment to data models
prescan.experiment.convertPexToDataModels;  % Convert the Prescan experiment files into MATLAB data models

% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file
% --------------------------------------------------------------------------------------------------------

% load vehicle configuration file
vec_info = readstruct("vehicle_configuration.xml");
vec_names = fieldnames(vec_info);
% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------
SimulinkFileName = pbFileName(1:end-3)+"_cs";
open_system(SimulinkFileName);

for r=1:length(vec_names)
    PhysicalCarsInExpDirectory{r} = SimulinkFileName + '/' + vec_names{r};
    blockPathSpeedProfile =  PhysicalCarsInExpDirectory{r} + "/SpeedProfile";
    % The Position parameter returns a four-element vector [left, top, right, bottom]
    SpeedProfilePosition = get_param(blockPathSpeedProfile, 'Position');
    
    ROSmsg2PrescanPos{r} = "ROSmsg2PrescanPos" + "_" + r;
    ROSmsg2PrescanPosPosition = [SpeedProfilePosition(1),SpeedProfilePosition(2)-130,SpeedProfilePosition(3),SpeedProfilePosition(4)-130];
    % MainLibrary can be found in 'C:\Program Files\MATLAB\R2022b\Custom_Prescan-ROS_Library'
    
    add_block('MainLibrary/ROSmsg2PrescanPos/ROS msg to Prescan Pos',...
        PhysicalCarsInExpDirectory{r} + "/" + ROSmsg2PrescanPos{r}, 'Position', ROSmsg2PrescanPosPosition);
    
    % Set the LinkStatus parameter to 'none' to disable the library link
    set_param(PhysicalCarsInExpDirectory{r} + "/" + ROSmsg2PrescanPos{r}, 'LinkStatus', 'none');
    
    Constant{r} = "TriggerPhysicalCar" + "_" + r ;
    
    % Calculate the height of the existing block
    % blockConstantHeight = SpeedProfilePosition(4) - SpeedProfilePosition(2);
    
    % Calculate the new block's position
    newBlockLeft = SpeedProfilePosition(1) + 50; % Shifting right by 20 units
    newBlockTop = SpeedProfilePosition(4)-145; % Placing it below the existing block with an offset equal to the block's height
    desiredWidth = 80;  % Define the desired width for the new block
    desiredHeight = 40; % Define the desired height for the new block
    ConstantPosition = [newBlockLeft, newBlockTop, newBlockLeft + desiredWidth, newBlockTop + desiredHeight];
    
    ConstantPosition = [ConstantPosition(1)+250,ConstantPosition(2),ConstantPosition(3)+250,ConstantPosition(4)];
    
    add_block('simulink/Commonly Used Blocks/Constant', PhysicalCarsInExpDirectory{r} + "/" +Constant{r},'position',ConstantPosition);
    set_param(PhysicalCarsInExpDirectory{r} + "/" + Constant{r} , 'Value', "TriggerCar_"+r)
    
    SwitchPosition = [SpeedProfilePosition(1)+450,SpeedProfilePosition(2)-70,SpeedProfilePosition(3)+450,SpeedProfilePosition(4)-70];
    Switch{r} = "Switch" + "_" + r ;
    
    add_block('simulink/Commonly Used Blocks/Switch', PhysicalCarsInExpDirectory{r} +"/"+ Switch{r},'position',SwitchPosition);
    
    delete_line(PhysicalCarsInExpDirectory{r}, "Path/1","STATE_"+vec_names{r}+"_rc/1");
    add_line(PhysicalCarsInExpDirectory{r},ROSmsg2PrescanPos{r}+"/1", Switch{r}+"/1")
    add_line(PhysicalCarsInExpDirectory{r},Constant{r}+"/1", Switch{r}+"/2")
    add_line(PhysicalCarsInExpDirectory{r},"Path/1", Switch{r}+"/3")
    add_line(PhysicalCarsInExpDirectory{r},Switch{r}+"/1", "STATE_"+vec_names{r}+"_rc/1")
    
    SubscriberBlockPath = PhysicalCarsInExpDirectory{r} +"/"+ROSmsg2PrescanPos{r}+"/Subsystem5/Subscribe";
    physical_id = vec_info.(vec_names{r}).physical_id;
    TopicName = "/vrpn_client_node/Car_"+physical_id+"_Tracking/pose";    % New topic name
    set_param(SubscriberBlockPath, 'TopicSource', 'Specify your own');
    set_param(SubscriberBlockPath, 'Topic', TopicName);
end

% change topic names accordingly
for i = 1:length(vec_names)
    dir = SimulinkFileName + '/' + vec_names{i} +"/"+ "ROSmsg2PrescanPos" + "_" + i ...
    +"/Subsystem5/";

    physical_id = vec_info.(vec_names{i}).physical_id;

    pose_block_path = dir + "Subscribe";
    TopicName = "/vrpn_client_node/Car_"+physical_id+"_Tracking/pose";
    set_param(pose_block_path, 'TopicSource', 'Select from ROS network');
    set_param(pose_block_path, 'Topic', TopicName);

    accel_block_path = dir + "Subscribe1";
    physical_id = 0; % IMPORTANT: twist and accel subscribers need to be 0
    TopicName = "/vrpn_client_node/Car_"+physical_id+"_Tracking/accel";
    set_param(accel_block_path, 'TopicSource', 'Select from ROS network');
    set_param(accel_block_path, 'Topic', TopicName);

    twist_block_path = dir + "Subscribe2";
    TopicName = "/vrpn_client_node/Car_"+physical_id+"_Tracking/twist";
    set_param(twist_block_path, 'TopicSource', 'Select from ROS network');
    set_param(twist_block_path, 'Topic', TopicName);
    
end

% --------------------------------------------------------------------------------------------------------
% Checking the status of ROS message (whether it has started being sent or not)
StatusDirectory = SimulinkFileName + '/' + vec_names{1};
StatusReceiverROS = "Status Receiver ROS";
StatusReceiverROSPosition = [ROSmsg2PrescanPosPosition(1),ROSmsg2PrescanPosPosition(2)-130,ROSmsg2PrescanPosPosition(3),ROSmsg2PrescanPosPosition(4)-130];
% MainLibrary can be found in 'C:\Program Files\MATLAB\R2022b\Custom_Prescan-ROS_Library'

add_block('MainLibrary/StatusReceiverROS/Status Receiver ROS',...
    StatusDirectory + "/" + StatusReceiverROS, 'Position', StatusReceiverROSPosition);

% Set the LinkStatus parameter to 'none' to disable the library link
set_param(StatusDirectory + "/" + StatusReceiverROS, 'LinkStatus', 'none');
% --------------------------------------------------------------------------------------------------------
