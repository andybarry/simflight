% Parameter identification for the deltawing plane

clear

%% setup
realtime_path = '/home/abarry/realtime/';
logfile_path = '/home/abarry/rlg/logs/2014-04-18-near-goalposts/mat/';

logfile_name = 'pass1-processed.mat';

% add log parsing scripts to path

addpath([realtime_path 'scripts/logs']);

% load data
load([logfile_path, logfile_name]);


%% convert inputs to model units

% convert servos

u = ConvertInputUnits(u);

%% trim to flight times only

[start_time, end_time] = FindActiveTimes(u.logtime, u.throttle_command, 1500);

assert(length(start_time) == 1, 'Number of active times ~= 1');

t_block = 0.2;
%end_time = start_time + t_block;


t_start = start_time : t_block : end_time;
t_end = start_time + t_block : t_block : end_time;

dt = 1/140; % approximate servo rate

%% setup comparison to model output for orientation PEM

dat = BuildIdDataRPY(est, u, t_start(1), t_end(1), dt);

file_name = 'tbsc_model_pem_wrapper';

order = [3, 3, 12];

initial_states = zeros(12,1); %x0;

parameters = [1; 1; 1];

nlgr = idnlgrey(file_name, order, parameters, initial_states, 0);

setinit(nlgr, 'Fixed', {false false false false false false false false false false false false});   % Estimate the initial state.

disp('Running pem...');
nlgr_fit = pem(dat, nlgr, 'Display', 'Full', 'MaxIter', 100);

disp('Simulating...');
%figure;
%compare(dat, nlgr_fit);
