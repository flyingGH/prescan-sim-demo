clearvars -except NUTS_defined
% --------------------------------------------------------------------------------------------------------
% Convert the Prescan experiment to data models
prescan.api.experiment.loadExperimentFromFile('PrescanDemoAMC.pb');  % Convert the Prescan experiment files into MATLAB data models

% Get the default filename for the Prescan experiment
pbFileName = prescan.experiment.getDefaultFilename;  % Get the default filename of the experiment

% Load the PB file into MATLAB so we can edit its data
experiment = prescan.api.experiment.loadExperimentFromFile(pbFileName);  % Load the experiment from the file
% -------------------------------------------------------------------------------------------------------
SimulinkFileName = pbFileName(1:end-3)+"_cs";
% -------------------------------------------------------------------------------------------------------
% Check if the model is loaded
if bdIsLoaded(SimulinkFileName)
    % Close the model without saving any changes
    close_system(SimulinkFileName, 0);
end

if exist(SimulinkFileName+".slx",'file') == 4
    delete(SimulinkFileName+".slx");
else
    disp('Please use the "Build" button in Prescan before running this script');
    return
end

CreateConfigurations

AirSensorMaxDetectableObjects

Virtual_Physical_Switch

Agent_Sensor

TrafficLight_Timers

TrafficLight_ROSmsg

% save changes and close
save_system;
close_system;
