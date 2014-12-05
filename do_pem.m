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

%% trim to flight times only and setup comparison to model output for orientation PEM

[start_time, end_time] = FindActiveTimes(u.logtime, u.throttle_command, 1500);

assert(length(start_time) == 1, 'Number of active times ~= 1');

t_block = 0.5;
%end_time = start_time + t_block;


t_start = start_time : t_block : end_time;
t_end = start_time + t_block : t_block : end_time;

dt = 1/140; % approximate servo rate

for i = 1 : min(length(t_start), length(t_end))
  %dat{i} = BuildIdDataRPYAirspeed(est, baro, u, t_start(i), t_end(i), dt);
  dat{i} = BuildIdDataRPY(est, u, t_start(i), t_end(i), dt);
end

%merged_dat = merge(dat{:});

merged_dat = merge(dat{2}, dat{3});

%% run prediction error minimization 


file_name = 'tbsc_model_pem_wrapper';

num_outputs = 3;
num_inputs = 3;
num_states = 12;

order = [num_outputs, num_inputs, num_states];

initial_states = [0 0 0 0 0 0 10 0 0 0 0 0]';

parameters = [1; 1; 1;];

nlgr = idnlgrey(file_name, order, parameters, initial_states, 0);

nlgr.InitialStates(1).Name = 'x';
nlgr.InitialStates(2).Name = 'y';
nlgr.InitialStates(3).Name = 'z';
nlgr.InitialStates(4).Name = 'roll';
nlgr.InitialStates(5).Name = 'pitch';
nlgr.InitialStates(6).Name = 'yaw';

nlgr.InitialStates(7).Name = 'U';
nlgr.InitialStates(8).Name = 'V';
nlgr.InitialStates(9).Name = 'W';
nlgr.InitialStates(10).Name = 'P';
nlgr.InitialStates(11).Name = 'Q';
nlgr.InitialStates(12).Name = 'R';


nlgr = setinit(nlgr, 'Fixed', {false false false false false false false false false false false false});   % Estimate the initial state.
%nlgr = setinit(nlgr, 'Minimum', {-100 -100 -100 -100 -100 -100 10 -100 -100 -100 -100 -100 });
%nlgr = setinit(nlgr, 'Maximum', {100 100 100 100 100 100 15 100 100 100 100 100 });

nlgr.InitialStates(7).Minimum = 10;
nlgr.InitialStates(7).Maximum = 15;

nlgr.InitialStates(8).Minimum = -0.5;
nlgr.InitialStates(8).Maximum = 0.5;

nlgr.InitialStates(9).Minimum = -0.5;
nlgr.InitialStates(9).Maximum = 0.5;

nlgr.InitialStates(10).Minimum = -1;
nlgr.InitialStates(10).Maximum = 1;

nlgr.InitialStates(11).Minimum = -1;
nlgr.InitialStates(11).Maximum = 1;

nlgr.InitialStates(12).Minimum = -1;
nlgr.InitialStates(12).Maximum = 1;

nlgr.Parameters(1).Minimum = 0.1;
nlgr.Parameters(1).Maximum = 10;

nlgr.Parameters(2).Minimum = 0.1;
nlgr.Parameters(2).Maximum = 10;

nlgr.Parameters(3).Minimum = 0.1;
nlgr.Parameters(3).Maximum = 10;

% nlgr.Parameters(4).Minimum = 0.1;
% nlgr.Parameters(4).Maximum = 10;
% 
% nlgr.Parameters(5).Minimum = 0.1;
% nlgr.Parameters(5).Maximum = 10;



disp('Running pem...');
nlgr_fit = pem(merged_dat, nlgr, 'Display', 'Full', 'MaxIter', 100);

%% display results

disp(' ------------- Initial States -------------');
DisplayNlgr(nlgr_fit.InitialStates);
disp(' ------------- Parameters  -------------');
DisplayNlgr(nlgr_fit.Parameters);



disp('Simulating...');
figure;
%[y_out, fit_out, x0_out] = compare(merged_dat, nlgr_fit);
compare(merged_dat, nlgr_fit);

disp('done.');
