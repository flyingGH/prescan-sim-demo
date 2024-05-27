% ==========================================================================================================
% this code can be used to create Simulink blocks to generate ROS messages
% of the detected objects (vehicles/pedestrians) by the sensors.
% it works for actors of type: cars, motors, busses and trailers
clearvars -except NUTS_defined

% ========================================= Scaling of the MAP =============================================
MapScaling = 10;
% ==========================================================================================================
% --------------------------------------------------------------------------------------------------------
% This conmmand is equvilanc of pressing "Build" from the GUI
% It writes the content of the GUI into the PB file
% So in refreshes your PB file and overwrite any previously made changes
% This is useful when multiple experiments are run one after the other and
% prevents changes from previous run still be present in current run

% Convert the Prescan experiment to data models
prescan.api.experiment.loadExperimentFromFile('PrescanDemoAMC.pb');  % Convert the Prescan experiment files into MATLAB data models

% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file
% --------------------------------------------------------------------------------------------------------

expStruct = experiment.getAsMatlabStruct();
AllObjectsName = cellfun(@(x) x.name, expStruct.worldmodel.object, 'UniformOutput', false);

% --------------------------------------------------------------------------------------------------------
AllObjectsUniqueId = cellfun(@(x) x.uniqueID, expStruct.worldmodel.object);

% load vehicle configuration file
vec_info = readstruct("vehicle_configuration.xml");
vec_names = fieldnames(vec_info);

PedestrianLibrary = {'Female','Male','Child','Adult','Boy','Toddler','Couple'};
PedestrianName = {};

for i = 1:length(PedestrianLibrary)
%     matches = AllObjectsName(contains(AllObjectsName, PedestrianLibrary{i}, 'IgnoreCase', true));
    matches = AllObjectsName(contains(AllObjectsName, PedestrianLibrary{i}, 'IgnoreCase', false));
    PedestrianName = [PedestrianName; matches];
end
PedestrianName = unique(PedestrianName); % remove duplicates

AllObjectsNameIDs = cellfun(@(x) x.numericalID, expStruct.worldmodel.object);
PedestrianIDs = cell(1,length(PedestrianName));
% Search for each name in PedestrianName and store the corresponding numericalID
for i = 1:numel(PedestrianName)
    idx = find(strcmp(PedestrianName{i}, AllObjectsName), 1);
    if ~isempty(idx)
        PedestrianIDs{i} = AllObjectsNameIDs(idx);
    end
end
PedestrianIdMatrix = double(cell2mat(PedestrianIDs));

% ActorsObjectUniqueId = double([expStruct.worldmodel.userObjectType{1, 1}.objectUniqueID,...
%     expStruct.worldmodel.userObjectType{2, 1}.objectUniqueID...
%     expStruct.worldmodel.userObjectType{3, 1}.objectUniqueID...
%     expStruct.worldmodel.userObjectType{5, 1}.objectUniqueID]);
% CarName = AllObjectsName(ismember(AllObjectsUniqueId, ActorsObjectUniqueId));

CarWithSensorId = zeros(1,length(expStruct.airsensormodel.sensor));
SensorName = cell(1,length(expStruct.airsensormodel.sensor));
for n=1:length(expStruct.airsensormodel.sensor)
    CarWithSensorId(n) = expStruct.airsensormodel.sensor{n,1}.sensorBase.worldObjectID;
    SensorName{n} = expStruct.airsensormodel.sensor{n,1}.sensorBase.name; 
end
CarWithSensorName = AllObjectsName(ismember(AllObjectsUniqueId, CarWithSensorId));

CarIds = double(expStruct.worldmodel.userObjectType{1,1}.objectUniqueID);
CarName = AllObjectsName(ismember(AllObjectsUniqueId, CarIds));
CarPrescanId = double(AllObjectsNameIDs(ismember(AllObjectsUniqueId, CarIds)));

AgentName = [arrayfun(@(x) ['car_' num2str(x)], 1:length(CarPrescanId), 'UniformOutput', false), ...
    arrayfun(@(x) ['pd_' num2str(x)], 1:length(PedestrianIdMatrix), 'UniformOutput', false)];

AgentIds = [CarPrescanId' PedestrianIdMatrix];

%Peparing the data for Simulink
NumberOfAgents = length(AgentIds);

% CarName = AllObjectsName(ismember(AllObjectsNameIDs, CarId));
% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------
SimulinkFileName = pbFileName(1:end-3)+"_cs";
open_system(SimulinkFileName);

% TrafficLightBlockPosition = cell(length(TrafficLight),1);
% SubsystemPosition = cell(length(TrafficLight),1);
% TrafficLight2ROSmessage = cell(length(TrafficLight),1);
% StringConstantPosition = cell(length(TrafficLight),1);
% StringConstant = cell(length(TrafficLight),1);
% FromWorkSpaceConstantPosition = cell(1,length(TrafficLight),1);
% FromWorkSpace = cell(length(TrafficLight),1);
% Subsystems = cell(length(TrafficLight),1);
for j=1:length(CarWithSensorName)
    % for j=1:1
    Directory{j} = SimulinkFileName + '/' + CarWithSensorName{j};
    blockPathAIRSesnsor =  Directory{j} + '/' + SensorName{j}+ "_Demux";
    
    % The Position parameter returns a four-element vector [left, top, right, bottom]
    AIRSesnsorPosition{j} = get_param(blockPathAIRSesnsor, 'Position');
    
    k = 1;
    for i=1:5
        shiftRightBy = 100;  % Change this based on your desired width
        DemuxPosition = [AIRSesnsorPosition{j}(3) + shiftRightBy, AIRSesnsorPosition{j}(2), AIRSesnsorPosition{j}(3)...
            + (AIRSesnsorPosition{j}(3) - AIRSesnsorPosition{j}(1)), AIRSesnsorPosition{j}(4)];
    
        Demux{i} = "Demux" + "_" + i;
    
        % Add a Demux block to the model at a specific position
        add_block('simulink/Signal Routing/Demux', Directory{j} + "/" + Demux{i},...
            'Position', DemuxPosition) % Add 200 to the y coordinates);
        set_param(Directory{j} + "/" + Demux{i}, 'Outputs', num2str(NumberOfAgents));
        % add_line(SimulinkFileName, Directory{j} + '/' + "AIR_"+(j+1)+"_Demux"+ "/1", Directory{j} + "/" + Demux{i} + "/1");
        if i == 3
            k = k+1;
        end
        % add_line("The Layer where the sensor block is","the name of the block from which you want to extract the signal from" +"/number of output port,
        % "the name of the block to which you want to connect the signal to +"/number of input port");
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},SensorName{j}+"_Demux"+ "/"+k, Demux{i} + "/1");
        k = k+1;
    end
    
    % for n=1:1
    for n=1:NumberOfAgents
        % DemuxPosition1 = [DemuxPosition(1)+30,DemuxPosition(2),DemuxPosition(3)+30,DemuxPosition(4)];
        
        shiftRightBy = 60;  % Change this based on your desired width
        FirstIfPosition = [AIRSesnsorPosition{j}(3) + shiftRightBy, AIRSesnsorPosition{j}(2), AIRSesnsorPosition{j}(3)...
            + (AIRSesnsorPosition{j}(3) - AIRSesnsorPosition{j}(1)), AIRSesnsorPosition{j}(4)];
        
        FirstIfPosition = [FirstIfPosition(1)+80,FirstIfPosition(2),FirstIfPosition(3)+80,FirstIfPosition(4)];
        
        FirstIf{n} = "First_if" + "_" + n;
        
        % Add an If block to the model.
        add_block('simulink/Ports & Subsystems/If', Directory{j} + "/" +FirstIf{n},'position',FirstIfPosition);
        
        % Set the 'Number of inputs' parameter
        % set_param(Directory{j} + "/" +FirstIf{n}, 'Inputs', '1');
        set_param(Directory{j} + "/" +FirstIf{n}, 'IfExpression', "u1 == AgentIds("+ num2str(1)+ ")");
        
        % Initialize an empty string for the 'Elseif expressions'.
        elseif_expr = '';
        
        % Loop through the matrix starting from the second element.
        for d = 2:NumberOfAgents
            % Construct the expression for each element.
            expr{d-1} = "u1 == AgentIds("+ num2str(d)+ ")";
        end
    
        numIndices = NumberOfAgents-1;
        
        % Generate the expressions based on the indices
        expr = cell(1, numIndices);
        for i = 1:numIndices
            expr{i} = sprintf('u1 == AgentIds(%d)', i+1);
        end
        
        % Join the expressions using a comma
        result = strjoin(expr, ',');
        
        % set_param(Directory{j} + "/" +FirstIf{n}, 'ElseIfExpressions', elseif_expr);
        set_param(Directory{j} + "/" +FirstIf{n}, 'ElseIfExpressions', result);
        
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Demux{3}+ "/"+n, FirstIf{n}+ "/1");
        
        IfActionPosition = [FirstIfPosition(1)+80,FirstIfPosition(2),FirstIfPosition(3)+80,FirstIfPosition(4)];
        
        Merge{n} = "Merge" + "_" + n;
        MergePosition = [IfActionPosition(1)+80,IfActionPosition(2),IfActionPosition(3)+80,IfActionPosition(4)];
        
        for l=1:NumberOfAgents
            if l ==1
                add_block('simulink/Signal Routing/Merge', Directory{j} + "/" + Merge{n},'position',MergePosition);
                set_param(Directory{j} + "/" + Merge{n}, 'Inputs', num2str(NumberOfAgents+1));
            end
    
            IfAction{n} = "If Action" + "_" + n + "_" + l;
            add_block('simulink/Ports & Subsystems/If Action Subsystem', Directory{j} + "/" +IfAction{n},'position',IfActionPosition);
            
            delete_line(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n},"In1/1","Out1/1");
            
            % Delete the input port
            delete_block(Directory{j} + "/" +IfAction{n}+"/In1");
            
            add_block('simulink/String/String Constant', Directory{j} + "/" +IfAction{n} + "/" + "StringConstant")
            
            % stringValue = ['"' char("Pedestrian") '"']; % Replacing the value with the actual string value
            % set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n} + "/" + "StringConstant" , 'String', ""+stringValue+"")
            
            stringValue = ['"' AgentName{l} '"']; % Replacing the value with the actual string value
            set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n} + "/" + "StringConstant" , 'String', stringValue)
            
            add_line(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n},"StringConstant"+ "/"+1, "Out1/1")
            
            add_line(SimulinkFileName+"/"+CarWithSensorName{j},FirstIf{n}+ "/"+l, IfAction{n}+"/Ifaction")
            
            add_line(SimulinkFileName+"/"+CarWithSensorName{j},IfAction{n}+"/1", Merge{n}+"/"+l)
            
            if l == NumberOfAgents
                IfActionPosition = [FirstIfPosition(1)+80,FirstIfPosition(2),FirstIfPosition(3)+80,FirstIfPosition(4)];
                IfAction{n} = "If Action" + "_" + n + "_" + (l+1);
                add_block('simulink/Ports & Subsystems/If Action Subsystem', Directory{j} + "/" +IfAction{n},'position',IfActionPosition);
                
                delete_line(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n},"In1/1","Out1/1");
                
                % Delete the input port
                delete_block(Directory{j} + "/" +IfAction{n}+"/In1");
                
                add_block('simulink/String/String Constant', Directory{j} + "/" +IfAction{n} + "/" + "StringConstant")
                
                % stringValue = ['"' char("NoPedestrian") '"']; % Replacing the value with the actual string value
                % set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n} + "/" + "StringConstant" , 'String', ""+stringValue+"")
                stringValue = ['"' char("0") '"'];
                set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n} + "/" + "StringConstant" , 'String', stringValue)
                
                add_line(SimulinkFileName+"/"+CarWithSensorName{j} + "/" +IfAction{n},"StringConstant"+ "/"+1, "Out1/1")
                add_line(SimulinkFileName+"/"+CarWithSensorName{j},FirstIf{n}+ "/"+(l+1), IfAction{n}+"/Ifaction")
                add_line(SimulinkFileName+"/"+CarWithSensorName{j},IfAction{n}+"/1", Merge{n}+"/"+(l+1))
            end
        end

        StringCompare{n} = "String Compare" + "_" + n; 
        StringComparePosition = [MergePosition(1)+150,MergePosition(2),MergePosition(3)+150,MergePosition(4)];
        add_block('simulink/String/String Compare', Directory{j} + "/" + StringCompare{n},'position',StringComparePosition);
        
        StringConstant{n} = "String Constant" + "_" + n; 
        StringConstantPosition = [StringComparePosition(1)-80,StringComparePosition(2)-100,StringComparePosition(3)-80,StringComparePosition(4)-100];
        add_block('simulink/String/String Constant', Directory{j} + "/" + StringConstant{n},'position',StringConstantPosition);
        stringValue = ['"' char("0") '"']; % Replacing the value with the actual string value
        set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" + StringConstant{n} , 'String', ""+stringValue+"")
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},StringConstant{n}+"/1", StringCompare{n}+"/1")
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Merge{n}+"/1", StringCompare{n}+"/2")
        
        SecondIf{n} = "Second_if" + "_" + n;
        SecondIfPosition = [StringComparePosition(1)+80,StringComparePosition(2),StringComparePosition(3)+80,StringComparePosition(4)];
        add_block('simulink/Ports & Subsystems/If', Directory{j} + "/" +SecondIf{n},'position',SecondIfPosition);
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},StringCompare{n}+"/1", SecondIf{n}+"/1")
        set_param(Directory{j} + "/" +SecondIf{n}, 'IfExpression', "u1 == 0");
        
        AddedTerminator{n} = "AddedTerminator" + "_" + n;
        AddedTerminatorPosition = [StringComparePosition(1)+80,StringComparePosition(2)/2,StringComparePosition(3)+80,StringComparePosition(4)/2];
        AddedTerminatorPosition = [AddedTerminatorPosition(1)+80,AddedTerminatorPosition(2)+270,AddedTerminatorPosition(3)+80,AddedTerminatorPosition(4)+270];
        add_block('simulink/Commonly Used Blocks/Terminator', Directory{j} + "/" +AddedTerminator{n},'position',AddedTerminatorPosition);
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},SecondIf{n}+"/2", AddedTerminator{n}+"/1")
        
        FirstDivide{n} = "FirstDivide" + "_" + n;
        
        % Calculate the height of the existing block
        blockHeight = AIRSesnsorPosition{j}(4) - AIRSesnsorPosition{j}(2);
        
        % Calculate the new block's position
        newBlockLeft = AIRSesnsorPosition{j}(1) + 400; % Shifting right by 20 units
        newBlockTop = AIRSesnsorPosition{j}(4) + blockHeight; % Placing it below the existing block with an offset equal to the block's height
        desiredWidth = 30;  % Define the desired width for the new block
        desiredHeight = 20; % Define the desired height for the new block
        FirstDividePosition = [newBlockLeft, newBlockTop, newBlockLeft + desiredWidth, newBlockTop + desiredHeight];
        % FirstDividePosition = [FirstDividePosition(1)+200,FirstDividePosition(2)+50,FirstDividePosition(3)+200,FirstDividePosition(4)+50];
        
        add_block('simulink/Math Operations/Divide', Directory{j} + "/" +FirstDivide{n},'position',FirstDividePosition);
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Demux{1}+ "/"+n, FirstDivide{n}+ "/1");
        
        SecondDivide{n} = "SecondDivide" + "_" + n;
        SecondDividePosition = [FirstDividePosition(1),FirstDividePosition(2)+30,FirstDividePosition(3),FirstDividePosition(4)+30];
        
        add_block('simulink/Math Operations/Divide', Directory{j} + "/" +SecondDivide{n},'position',SecondDividePosition);
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Demux{4}+ "/"+n, SecondDivide{n}+ "/1");
        
        FirstConstant{n} = "FirstConstant" + "_" + n;
        
        % Calculate the height of the existing block
        blockHeight = AIRSesnsorPosition{j}(4) - AIRSesnsorPosition{j}(2);
        
        % Calculate the new block's position
        newBlockLeft = AIRSesnsorPosition{j}(1) + 150; % Shifting right by 20 units
        newBlockTop = AIRSesnsorPosition{j}(4) + blockHeight; % Placing it below the existing block with an offset equal to the block's height
        desiredWidth = 30;  % Define the desired width for the new block
        desiredHeight = 20; % Define the desired height for the new block
        FirstConstantPosition = [newBlockLeft, newBlockTop, newBlockLeft + desiredWidth, newBlockTop + desiredHeight];
        
        FirstConstantPosition = [FirstConstantPosition(1)+100,FirstConstantPosition(2)+5,FirstConstantPosition(3)+100,FirstConstantPosition(4)+5];
        
        add_block('simulink/Commonly Used Blocks/Constant', Directory{j} + "/" +FirstConstant{n},'position',FirstConstantPosition);
        set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" + FirstConstant{n} , 'Value', num2str(MapScaling))
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},FirstConstant{n}+"/1", FirstDivide{n}+"/2")
        
        SecondConstant{n} = "SecondConstant" + "_" + n;
        SecondConstantPosition = [FirstConstantPosition(1),FirstConstantPosition(2)+30,FirstConstantPosition(3),FirstConstantPosition(4)+30];
        
        add_block('simulink/Commonly Used Blocks/Constant', Directory{j} + "/" +SecondConstant{n},'position',SecondConstantPosition);
        set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" + SecondConstant{n} , 'Value',num2str(MapScaling))
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},SecondConstant{n}+"/1", SecondDivide{n}+"/2")
        
        MatrixConcatenate{n} = "MatrixConcatenate" + "_" + n;
        MatrixConcatenatePosition = [IfActionPosition(1)+150,IfActionPosition(2)+230,IfActionPosition(3)+150,IfActionPosition(4)+235];
        
        add_block('simulink/Math Operations/Matrix Concatenate', Directory{j} + "/" +MatrixConcatenate{n},'position',MatrixConcatenatePosition);
        set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" + MatrixConcatenate{n}, 'NumInputs', '4'); % For 4 inputs
        set_param(SimulinkFileName+"/"+CarWithSensorName{j} + "/" + MatrixConcatenate{n}, 'ConcatenateDimension', '1'); % Concatenate along rows (vertically)
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},FirstDivide{n}+"/1", MatrixConcatenate{n}+"/1")
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},SecondDivide{n}+"/1", MatrixConcatenate{n}+"/2")
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Demux{2}+ "/"+n, MatrixConcatenate{n}+ "/3");
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Demux{5}+ "/"+n, MatrixConcatenate{n}+ "/4");
        
        Sensor2ROSmessage{n} = "Sensor2ROSmessage" + "_" + n;
        Sensor2ROSmessagePosition = [SecondIfPosition(1)+250,SecondIfPosition(2)+100,SecondIfPosition(3)+250,SecondIfPosition(4)+100];
        % MainLibrary can be found in 'C:\Program Files\MATLAB\R2022b\Custom_Prescan-ROS_Library'
        
        add_block('MainLibrary/Sensor2ROSmessageAgent/Sensor ROS Message Agent',...
            Directory{j} + "/" + Sensor2ROSmessage{n}, 'Position', Sensor2ROSmessagePosition);
        
        % Set the LinkStatus parameter to 'none' to disable the library link
        set_param( Directory{j} + "/" + Sensor2ROSmessage{n}, 'LinkStatus', 'none');
        
        % add_block('MainLibrary/Sensor2ROSmessageModified/Sensor ROS Message',...
        %     Directory{j} + "/" + Sensor2ROSmessage{n}, 'Position', Sensor2ROSmessagePosition);
        
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},SecondIf{n}+ "/1", Sensor2ROSmessage{n}+"/Ifaction")
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},Merge{n}+ "/"+1, Sensor2ROSmessage{n}+"/1")
        add_line(SimulinkFileName+"/"+CarWithSensorName{j},MatrixConcatenate{n}+"/1", Sensor2ROSmessage{n}+"/2")
        
        blockPath = SimulinkFileName+"/"+CarWithSensorName{j} +"/"+Sensor2ROSmessage{n}+"/Subsystem1/Publish2";
        physical_id = vec_info.(CarWithSensorName{j}).physical_id;
        TopicName = "/car_"+physical_id+"/sensor";    % New topic name
        set_param(blockPath, 'TopicSource', 'Specify your own');
        set_param(blockPath, 'Topic', TopicName);
    end

Ez = "MMR";
end
% ----------------------------------------------------------------------------------------------------------------------------
