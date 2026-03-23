% Adds the project source folders to the MATLAB path.
project_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(project_dir, 'src')));
disp('MATLAB path updated for project sources.');
