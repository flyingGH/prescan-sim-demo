clearvars -except NUTS_defined; clc
% --------------------------------------------------------------------------------------------------------
% This conmmand is equvilanc of pressing "Build" from the GUI
% It writes the content of the GUI into the PB file
% So in refreshes your PB file and overwrite any previously made changes
% This is useful when multiple experiments are run one after the other and
% prevents che changes from previous run co still be present in current run

% Convert the Prescan experiment to data models
prescan.experiment.convertPexToDataModels;  % Convert the Prescan experiment files into MATLAB data models

% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file
% --------------------------------------------------------------------------------------------------------   
expStruct = experiment.getAsMatlabStruct();
AllObjectsUniqueId = cellfun(@(x) x.uniqueID, expStruct.worldmodel.object);
AllObjectsName = cellfun(@(x) x.name, expStruct.worldmodel.object, 'UniformOutput', false);

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
for i=1:length(CarWithSensorName)
    % Get the object from the experiment by its name
    Car = experiment.getObjectByName(CarWithSensorName{i});
    AudiAir = prescan.api.air.getAttachedAirSensors(Car);
    AudiAir.maxDetectableObjects = NumberOfAgents;
end
% save_system
experiment.saveToFile(strcat(pbFileName))
% SimulinkFileName = pbFileName(1:end-3)+"_cs";
prescan.api.simulink.generate
