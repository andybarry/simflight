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

% delay in ms from command to execution
delay_ms = 20;

use_airspeed = false;

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
  airspeed_dat{i} = BuildIdDataRPYAirspeed(est, baro, u, t_start(i), t_end(i), dt, delay_ms);
  dat{i} = BuildIdDataRPY(est, u, t_start(i), t_end(i), dt, delay_ms);
  
  
end

if use_airspeed
    dat = airspeed_dat;
   
end
  

merge_nums = [2 ,3, 4, 5];

%merged_dat = merge(dat{:});

%merged_dat = merge(dat{3}, dat{4}, dat{5}, dat{6});
merged_dat = merge(dat{merge_nums});

merged_airspeed_dat = merge(airspeed_dat{merge_nums});
%merged_dat = dat{2};

%% run prediction error minimization 


file_name = 'tbsc_model_pem_wrapper';

if use_airspeed
  num_outputs = 4;
else
  num_outputs = 3;
end

num_inputs = 3;
num_states = 12;

order = [num_outputs, num_inputs, num_states];

%initial_states = repmat([0 0 0 0 0 0 10 0 0 0 0 0]', 1, 2);

% extract inital state guesses from the data
for i = 1 : length(merged_airspeed_dat.OutputData)
  
  x0_dat{i} = merged_airspeed_dat.OutputData{i}(1,:);
  
end

x0_dat_full{1} = zeros( 1, length(merged_dat.OutputData));
x0_dat_full{2} = zeros( 1, length(merged_dat.OutputData));
x0_dat_full{3} = zeros( 1, length(merged_dat.OutputData));

x0_dat_full{4} = [];
x0_dat_full{5} = [];
x0_dat_full{6} = [];
x0_dat_full{7} = [];

for i = 1 : length(x0_dat)
  x0_dat_full{4} = [ x0_dat_full{4} x0_dat{i}(1) ];
  x0_dat_full{5} = [ x0_dat_full{5} x0_dat{i}(2) ];
  x0_dat_full{6} = [ x0_dat_full{6} x0_dat{i}(3) ];
  x0_dat_full{7} = [ x0_dat_full{7} x0_dat{i}(4) ];
end

%x0_dat_full{7} = [ 9.9607 11.3508 ];%zeros( 1, length(merged_dat.OutputData));

x0_dat_full{8} = zeros( 1, length(merged_dat.OutputData));
x0_dat_full{9} = zeros( 1, length(merged_dat.OutputData));

x0_dat_full{10} = zeros( 1, length(merged_dat.OutputData));
x0_dat_full{11} = zeros( 1, length(merged_dat.OutputData));
x0_dat_full{12} = zeros( 1, length(merged_dat.OutputData));

  


%initial_states = { [0 0] [0 0] [0 0] [0 0] [0 0] [0 0] [10 10] [0 0] [0 0] [0 0] [0 0] [0 0] };

parameters = [1; 1; 1; 1; 1];

nlgr = idnlgrey(file_name, order, parameters, x0_dat_full);

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


nlgr = setinit(nlgr, 'Fixed', {true true true true true true true false false false false false });   % Estimate the initial state.
%nlgr.InitialStates(1).Fixed = [false false false false false false false false false false false false];
%nlgr.InitialStates(2).Fixed = [false false false false false false false false false false false false];
% nlgr = setinit(nlgr, 'Minimum', {-100 -100 -100 -100 -100 -100 9 -1 -1 -5 -5 -5 });
% nlgr = setinit(nlgr, 'Maximum', {100 100 100 100 100 100 15 1 1 5 5 5 });

% nlgr.InitialStates(7).Minimum = 9;
% nlgr.InitialStates(7).Maximum = 15;
% 
% nlgr.InitialStates(8).Minimum = -3;
% nlgr.InitialStates(8).Maximum = 3;
% 
% nlgr.InitialStates(9).Minimum = -5;
% nlgr.InitialStates(9).Maximum = 10;
% 
% nlgr.InitialStates(10).Minimum = -5;
% nlgr.InitialStates(10).Maximum = 5;
% 
% nlgr.InitialStates(11).Minimum = -5;
% nlgr.InitialStates(11).Maximum = 5;
% 
% nlgr.InitialStates(12).Minimum = -5;
% nlgr.InitialStates(12).Maximum = 5;

%% estimate initial states
% 
% disp('Estimating initial states...');
% 
% 
% nlgr.Algorithm.Display = 'full';
% 
% %x0 = zeros(num_states, length(merge_nums));
% 
% x0 = findstates(nlgr, merged_dat, x0_dat_full)
% % x0 = [0 0
% %          0         0
% %          0         0
% %    -0.1972   -0.2760
% %     0.4608    0.4951
% %    -0.1373   -0.1575
% %     9.1312    9.9785
% %    -0.1730   -0.0407
% %     1.5799    1.0444
% %    -0.5888   -0.7839
% %     0.1672    0.8154
% %    -0.0247   -0.3423];
% 
%  for i = 1 : length(merge_nums)
%    
% 
%    for j = 1 : num_states
%      nlgr.InitialStates(j).Value(i) = x0(j,i);
%    end
%  
%  end
% 
% 
% 
% nlgr = setinit(nlgr, 'Fixed', {true true true true true true true true true true true true});

%%


%nlgr.Algorithm.Regularization.Lambda = 0.01; % use regularization
%nlgr.Algorithm.Regularization.Nominal = 'model'; % attempt to keep parameters close to initial guesses
%  
% RR = diag([ones(1,length(parameters)) 0.001*ones(1,length(z.ExperimentName)*12)]);
% % RR(7,7) = RR(7,7)*10;
% nlgr.Algorithm.Regularization.R = RR;


% nlgr.Parameters(1).Minimum = 0.1;
% nlgr.Parameters(1).Maximum = 10;
% 
% nlgr.Parameters(2).Minimum = 0.1;
% nlgr.Parameters(2).Maximum = 10;
% 
% nlgr.Parameters(3).Minimum = 0.1;
% nlgr.Parameters(3).Maximum = 10;

% nlgr.Parameters(4).Minimum = 0.1;
% nlgr.Parameters(4).Maximum = 10;
% 
% nlgr.Parameters(5).Minimum = 0.1;
% nlgr.Parameters(5).Maximum = 10;

%% plot data

disp('Plotting data...');
figure(1)
clf
plot(merged_dat);
%% run pem

disp('Running pem...');
nlgr_fit = pem(merged_dat, nlgr, 'Display', 'Full', 'MaxIter', 20);

%% display results

disp(' ------------- Initial States -------------');
DisplayNlgr(nlgr_fit.InitialStates);
disp(' ------------- Parameters  -------------');
DisplayNlgr(nlgr_fit.Parameters);



disp('Simulating...');


% get initial conditions

for i = 1 : num_states
  
  x0_out(i,:) = nlgr_fit.InitialStates(i).Value;
  
end

compare_options = compareOptions('InitialCondition',x0_out);

%[y_out, fit_out, x0_out] = compare(merged_dat, nlgr_fit);
figure;
compare(merged_dat, nlgr_fit, compare_options);

disp('done.');
