clc;
addpath(genpath('../../src'));
disp('Creating configuration files...');
CreateConfigurations;
disp('Configuration files created.');
disp('Updating models...');
disp('This process may take a few minute...');
UpdateModel;
disp('Model updated successfully!');
disp('Running the experiment...');
%%%%%%%%%%%%%%% MODIFY HERE (START) %%%%%%%%%%%%%%%
 
% set target linux computer name (client), as defined by the windows 'HOSTS' file
target_name = 'f1tenth'; % MODIFY HERE, Remember to add this on the host's file
 
% set the iP of this computer (host computer)
host_ip = '10.0.0.194';  % MODIFY HERE
%%%%%%%%%%%%%%% MODIFY HERE (END) %%%%%%%%%%%%%%%
runExperiment(target_name, host_ip);