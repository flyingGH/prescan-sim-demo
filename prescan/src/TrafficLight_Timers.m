% This file creates all the necessary timers (stair generators) for 
% each traffic light according to the user's configuration file. 
% The traffic light colors are managed by manipulating the brightness
% of each light of every traffic light to produce a meaningful color
% cycle (red, green, orange).

% For now, the traffic light configuration file is created manually 
% and can be used as a template for future prescan experiments. In the
% future, the configuration file will be generated automatically during 
% the building process.

%% Initial configuration (extract prescan experiment object file)

% Convert the Prescan experiment to data models
prescan.api.experiment.loadExperimentFromFile('PrescanDemoAMC.pb');  % Convert the Prescan experiment files into MATLAB data models

% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file
% --------------------------------------------------------------------------------------------------------

SimulinkFileName = pbFileName(1:end-3)+"_cs";

%% Add times according to configuration file

%SimulinkFileName = 'test_traffic_lights'; % temporary
open_system(SimulinkFileName)

% read traffic light XML configuration file
tl_info = readstruct("traffic_configuration.xml");

% extract general traffic light information
brightness_on = tl_info.general.brightness_on;
brightness_off = tl_info.general.brightness_off;
duration_orange = tl_info.general.duration_orange;

% for each traffic light present in the configuration file
tl_names = fieldnames(tl_info.objects);
for tl=1:numel(tl_names)
    
    % extract specific traffic light information
    name = tl_names{tl};
    cycle_duration = tl_info.objects.(name).cycle_duration;
    ratio_red = tl_info.objects.(name).ratio_red;
    ratio_green = tl_info.objects.(name).ratio_green;

    % construct time array (Red->Green->Orange)
    timings = zeros(1, 4);
    timings(2) = timings(1) + (cycle_duration-duration_orange)*ratio_red;
    timings(3) = timings(2) + (cycle_duration-duration_orange)*ratio_green;
    timings(4) = timings(3) + duration_orange;

    % for each traffic light state
    state_names = ["Red", "Green", "Orange"];
    for s=1:length(state_names)
        
        % construct amplitude array
        amplitude = ones(1, 4)*brightness_off;
        amplitude(s) = brightness_on;

        % get position of color mux block
        current_dir = SimulinkFileName+"/"+name+"/";
        mux_name = state_names(s)+"_Mux";
        mux_path = current_dir+mux_name;
        mux_pos = get_param(mux_path, 'Position');
        
        % create stair generator with parameters
        height = 50;
        offset_x = -130;
        offset_y = -80;
        stair_pos = [mux_pos(1) + offset_x, (mux_pos(2)+mux_pos(4))/2 - height/2 + offset_y, mux_pos(3) + offset_x, (mux_pos(2)+mux_pos(4))/2 + height/2 + offset_y];
        block_type = 'ee_sl_lib/General Control/Stair Generator';
        block_name = "stair_generator_"+tl+"_"+s;
        block_path = current_dir+block_name;
        add_block(block_type, block_path, ...
            'TimeInput', "["+sprintf('%.2f %.2f %.2f %.2f', timings)+"]", ...
            'AmplitudeInput', "["+sprintf('%.2f %.2f %.2f %.2f', amplitude)+"]", ...
            'RepeatPattern', 'on', ...
            'Position', stair_pos);
        

        % remove previously connected blocks and lines
        delete_line(current_dir, state_names(s)+"_mscd/1", mux_name+"/1");
        delete_block(current_dir+state_names(s)+"_mscd");

        % connect stair generator to brightness of traffic light block
        add_line(current_dir, block_name+"/1", mux_name+"/1")
    end
end


