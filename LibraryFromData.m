%% load data

clear

%% load trajectories we like already

trajectory_library = 'traj-archive/basic-lib.mat';
disp(['Loading ' trajectory_library '...']);
load(trajectory_library);


%%

date = '2015-09-17';
name = 'field-test';
log_number = '04';
hostname = 'odroid-gps2';


dir = [date '-' name '/' hostname '/'];
filename = ['lcmlog_' strrep(date, '-', '_') '_' log_number '.mat'];


dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');

disp(['Loading ' filename '...']);

loadDeltawing

disp('done.');

%% extract time

start_t = 127.629;
end_t = 131.0;

lib = AddTrajectoryFromData(lib, est, u, start_t, end_t, 'right-turn-from-data');