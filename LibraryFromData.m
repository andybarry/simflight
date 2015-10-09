%% load data

clear

%% load trajectories we like already

trajectory_library = 'traj-archive/basic-lib.mat';
disp(['Loading ' trajectory_library '...']);
load(trajectory_library);


%%

date = '2015-10-08';
name = 'field-test';
log_number = '07';
hostname = 'odroid-gps3';


dir = [date '-' name '/' hostname '/'];
filename = ['lcmlog_' strrep(date, '-', '_') '_' log_number '.mat'];


dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');

disp(['Loading ' filename '...']);

loadDeltawing

disp('done.');

%% extract trajectories

start_t = 140.2;
end_t = 142.7;

lib = AddTrajectoryFromData(lib, est, u, start_t, end_t, 'left-jog-from-data');

start_t = 135.24;
end_t = 137.7;

lib = AddTrajectoryFromData(lib, est, u, start_t, end_t, 'right-jog-from-data');