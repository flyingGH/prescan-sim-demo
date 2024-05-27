clearvars -except NUTS_defined
% --------------------------------------------------------------------------------------------------------
% Convert the Prescan experiment to data models
prescan.api.experiment.loadExperimentFromFile('PrescanDemoAMC.pb');  % Convert the Prescan experiment files into MATLAB data models


% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file


%% Create Traffic Light Configuration

% settings
dummy_virtual = false;
dummy_physical_id = 3;

% extract all traffic light objects
exp_obj = experiment.getAsMatlabStruct();
exp_object_names = cellfun(@(x) x.name, exp_obj.worldmodel.object, 'UniformOutput', false);
exp_object_types = cellfun(@(x) x.modelFile, exp_obj.worldmodel.object, 'UniformOutput', false);
new_vehicles = string(exp_object_names(contains(exp_object_types, 'Vehicles', 'IgnoreCase', false)));

% create empty struct
vec_info = struct();

configuration_filename = 'vehicle_configuration.xml';
% check if there is an existing XML file
if exist(configuration_filename, 'file')
    % fill struct
    vec_info = readstruct(configuration_filename);
    existing_vehicles = fieldnames(vec_info);

    % add missing entries to existing configuration file if necessary
    vec_to_add = setdiff(new_vehicles, existing_vehicles);
    for vec = 1:length(vec_to_add)
        % fill up with dummy data
        vec_info.(vec_to_add(vec)).virtual = dummy_virtual;
        vec_info.(vec_to_add(vec)).physical_id = dummy_physical_id;
    end

    % remove missing entries from existing configuration file if necessary
    vec_to_remove = setdiff(existing_vehicles, new_vehicles);
    for vec = 1:length(vec_to_remove)
        vec_info = rmfield(vec_info, vec_to_remove(vec));
    end
else    % no existing XML file, create from scratch
    
    % fill up object data
    for vec = 1:length(new_vehicles)
        vec_info.(new_vehicles(vec)) = struct();
        vec_info.(new_vehicles(vec)).virtual = dummy_virtual;
        vec_info.(new_vehicles(vec)).physical_id = dummy_physical_id;
    end
end

% write final struct (back) to XML file
writestruct(vec_info, configuration_filename);