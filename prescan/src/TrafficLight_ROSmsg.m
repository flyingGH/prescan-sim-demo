clc;
% --------------------------------------------------------------------------------------------------------
% This conmmand is the equvilance of pressing "Build" from the GUI
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

% ========================================= Scaling of the MAP =============================================
MapScaling = 10;
% ==========================================================================================================

expStruct = experiment.getAsMatlabStruct();
AllObjectsName = cellfun(@(x) x.name, expStruct.worldmodel.object, 'UniformOutput', false);

% --------------------------------------------------------------------------------------------------------
TrafficLight = AllObjectsName(contains(AllObjectsName, 'RoadsideLight', 'IgnoreCase', true));

TrafficLightPosition = cell(1,length(TrafficLight));
for i=1:length(TrafficLight)
    TrafficLightPosition{i} = experiment.getObjectByName(TrafficLight{i}).pose.position;
end
% Convert the struct into a matrix and add a zero at the beginning of each matrix
TrafficLightPosition_Mat = cellfun(@(pos) [0, pos.x, pos.y, pos.z], TrafficLightPosition, 'UniformOutput', false);
% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------
SimulinkFileName = pbFileName(1:end-3)+"_cs";
open_system(SimulinkFileName);

TrafficLightBlockPosition = cell(length(TrafficLight),1);
SubsystemPosition = cell(length(TrafficLight),1);
TrafficLight2ROSmessage = cell(length(TrafficLight),1);
StringConstantPosition = cell(length(TrafficLight),1);
StringConstant = cell(length(TrafficLight),1);
FromWorkSpaceConstantPosition = cell(1,length(TrafficLight),1);
FromWorkSpace = cell(length(TrafficLight),1);
Subsystems = cell(length(TrafficLight),1);

for n=1:length(TrafficLight)
blockPath =  SimulinkFileName + '/' + TrafficLight{n};

% The Position parameter returns a four-element vector [left, top, right, bottom]
TrafficLightBlockPosition{n} = get_param(blockPath, 'Position');

% Desired shift to the right
shiftRight = 400;

% % Calculate the new position
width = TrafficLightBlockPosition{n}(3) - TrafficLightBlockPosition{n}(1);
height = TrafficLightBlockPosition{n}(4) - TrafficLightBlockPosition{n}(2);

% New position computation
SubsystemPosition{n} = [
    TrafficLightBlockPosition{n}(1) + width + shiftRight;  % Shift 600 to the right
    TrafficLightBlockPosition{n}(2) - 0.15 * height;    % 15% higher
%     existingBlockPosition(3) + width + shiftRight + 2*width, % Twice the width
    TrafficLightBlockPosition{n}(3) + width + shiftRight + 0.5*width; % Twice the width
    TrafficLightBlockPosition{n}(4) - 0.15 * height + height  % Twice the height
];

TrafficLight2ROSmessage{n} = "TrafficLight2ROSmessage" + "_" + n;

% MainLibrary can be found in 'C:\Program Files\MATLAB\R2022b\Custom_Prescan-ROS_Library'
add_block('MainLibrary/TrafficLight2ROSmessage/Traffic Light ROS Message',...
    SimulinkFileName + "/" + TrafficLight2ROSmessage{n}, 'Position', SubsystemPosition{n});

% Set the LinkStatus parameter to 'none' to disable the library link
set_param(SimulinkFileName + "/" + TrafficLight2ROSmessage{n}, 'LinkStatus', 'none');

add_line(SimulinkFileName, TrafficLight{n} + "/1", TrafficLight2ROSmessage{n} + "/1");
add_line(SimulinkFileName, TrafficLight{n} + "/2", TrafficLight2ROSmessage{n} + "/2");
add_line(SimulinkFileName, TrafficLight{n} + "/3", TrafficLight2ROSmessage{n} + "/3");

% New position calculation
StringConstantPosition{n} = [TrafficLightBlockPosition{n}(1) + 300, TrafficLightBlockPosition{n}(2) + 200,...
    TrafficLightBlockPosition{n}(1) + 300 + 0.8*(TrafficLightBlockPosition{n}(3)-TrafficLightBlockPosition{n}(1)),...
    TrafficLightBlockPosition{n}(2) + 200 + (TrafficLightBlockPosition{n}(4)-TrafficLightBlockPosition{n}(2))/4];

StringConstantPosition{n} = StringConstantPosition{n} + [0, 25, 0, 25];

StringConstant{n} = "StringConstant" + "_" + n;

% Add a String Constant block to the model at a specific position
add_block('simulink/String/String Constant', SimulinkFileName + "/" + StringConstant{n},...
    'Position', StringConstantPosition{n}) % Add 200 to the y coordinates);

% Set the value of the String Constant block
% stringValue = TrafficLight{n}; % Replacing the value with the actual string value
stringValue = ['"' TrafficLight{n} '"']; % Replacing the value with the actual string value
set_param(SimulinkFileName + "/" + StringConstant{n} , 'String', ""+stringValue+"");

% StringConstant{n} = strrep(StringConstant{n}, '/', '');

add_line(SimulinkFileName, StringConstant{n}+"/1", TrafficLight2ROSmessage{n} + "/4");

FromWorkSpaceConstantPosition{n} = StringConstantPosition{n} +[0, 80, 0, 80];

FromWorkSpace{n} = "FromWorkspace" + "_" + n;

% Add a 'From Workspace' block to the model at a specific position
add_block('simulink/Sources/From Workspace', SimulinkFileName + "/" + FromWorkSpace{n}, 'Position', FromWorkSpaceConstantPosition{n});

% Set the variable name for the 'From Workspace' block
% varName = "TrafficLightPosition_Mat{"+n+"}"; % Replacing the value with actual variable name
varName = num2str(TrafficLightPosition_Mat{n}/MapScaling); % Replacing the value with actual variable name
% Replace spaces with commas
varNameComma = strrep(varName, ' ', ',');

% Remove consecutive commas (resulting from multiple spaces)
varNameSingleComma = regexprep(varNameComma, ',+', ',');

% Add brackets
varNameFormatted = ['[' varNameSingleComma ']'];

set_param(SimulinkFileName + "/" + FromWorkSpace{n}, 'VariableName', varNameFormatted);

% FromWorkSpace{n} = strrep(FromWorkSpace{n}, '/', '');

add_line(SimulinkFileName, FromWorkSpace{n}+"/1", TrafficLight2ROSmessage{n} + "/5");

% Desired shift to the right
shiftRight = 400;

% Assume n is defined
% TrafficLightBlockPosition{n} = [left, bottom, right, top];

% Extracting original position for readability
left   = TrafficLightBlockPosition{n}(1);
bottom = TrafficLightBlockPosition{n}(2);
right  = TrafficLightBlockPosition{n}(3);
top    = TrafficLightBlockPosition{n}(4);

% Width and height
originalWidth  = right - left;
originalHeight = top - bottom;

% New width and height (half the original)
newWidth  = originalWidth / 2;
newHeight = originalHeight / 2;

% Update the position
SubsystemPosition{n} = [
    left + shiftRight,           % Shifted right x-coordinate
    bottom,                      % Same bottom y-coordinate
    left + shiftRight + newWidth,   % New right x-coordinate
    bottom + newHeight           % New top y-coordinate
];

% Subsystems{n} = "Subsystem" + "_" + n;
% add_block('built-in/Subsystem',SimulinkFileName + "/" + Subsystems{n},'Position',SubsystemPosition{n})

% blocksToGroup = {TrafficLight2ROSmessage{n}};
% % blocksToGroup = {TrafficLight2ROSmessage{n}, StringConstant{n},FromWorkSpace{n}};
% Simulink.BlockDiagram.copyContentsToSubsystem...
% (blocksToGroup,SimulinkFileName + "/" + Subsystems{n})

% bh = Simulink.findBlocks(SimulinkFileName + "/" + TrafficLight2ROSmessage{n});
% Simulink.BlockDiagram.createSubsystem(bh);

% blockNames = {
%     [SimulinkFileName, '/', TrafficLight2ROSmessage{n}];
%     [SimulinkFileName, '/', StringConstant{n}];
%     [SimulinkFileName, '/', FromWorkSpace{n}]
% };
% 
% % Create subsystem
% Simulink.BlockDiagram.createSubsystem(blockNames);

end
% --------------------------------------------------------------------------------------------------------
