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
dummy_cycle_duration = 10;
dummy_duration_orange = 0.5;
dummy_brightness_on = 1.841645;
dummy_brightness_off = 0.0;
dummy_ratio_red = 0.5;
dummy_ratio_green = 0.5;

% extract all traffic light objects
exp_obj = experiment.getAsMatlabStruct();
exp_object_names = cellfun(@(x) x.name, exp_obj.worldmodel.object, 'UniformOutput', false);
new_tl = string(exp_object_names(contains(exp_object_names, 'Light', 'IgnoreCase', false)));

% create empty struct
tl_info = struct();

configuration_filename = 'traffic_configuration.xml';
% check if there is an existing XML file
if exist(configuration_filename, 'file')
    % fill struct
    tl_info = readstruct(configuration_filename);
    existing_tl = fieldnames(tl_info.objects);

    % add missing entries to existing configuration file if necessary
    tl_to_add = setdiff(new_tl, existing_tl);
    for tl = 1:length(tl_to_add)
        % fill up with dummy data
        tl_info.objects.(tl_to_add(tl)).cycle_duration = dummy_cycle_duration;
        tl_info.objects.(tl_to_add(tl)).ratio_red = dummy_ratio_red;
        tl_info.objects.(tl_to_add(tl)).ratio_green = dummy_ratio_green;
    end

    % remove missing entries from existing configuration file if necessary
    tl_to_remove = setdiff(existing_tl, new_tl);
    for tl = 1:length(tl_to_remove)
        tl_info.objects = rmfield(tl_info.objects, tl_to_remove(tl));
    end
else    % no existing XML file, create from scratch
    
    % fill up general data
    tl_info.general.brightness_on = dummy_brightness_on;
    tl_info.general.brightness_off = dummy_brightness_off;
    tl_info.general.duration_orange = dummy_duration_orange;

    % fill up object data
    tl_info.objects = struct();
    for tl = 1:length(new_tl)
        tl_info.objects.(new_tl(tl)).cycle_duration = dummy_cycle_duration;
        tl_info.objects.(new_tl(tl)).ratio_red = dummy_ratio_red;
        tl_info.objects.(new_tl(tl)).ratio_green = dummy_ratio_green;
    end
end

% write final struct (back) to XML file
writestruct(tl_info, configuration_filename);