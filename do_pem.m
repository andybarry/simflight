% Parameter identification for the deltawing plane

clear

%% setup
realtime_path = '/home/abarry/realtime/';
logfile_path = '/home/abarry/rlg/logs/2015-03-31-field-test/gps-logs/';

logfile_name = 'lcmlog_2015_03_31_11.mat';

% add log parsing scripts to path

addpath([realtime_path 'scripts/logs']);

% load data
dir = logfile_path;
filename = logfile_name;

loadDeltawing

% delay in ms from command to execution

% delay is zero because we are using servo_out, which is the message after
% it has come back from the APM to the CPU

delay_ms = 20;
%warning(['delay = ' num2str(delay_ms) ' ms']);



use_airspeed = true;

%% trim to flight times only and setup comparison to model output for orientation PEM

[start_time, end_time] = FindActiveTimes(est.logtime, est.pos.z, 9.0);

assert(length(start_time) == 1, 'Number of active times ~= 1');

%% temp %%

start_time = start_time + 50;
%end_time = start_time + 20;

%%

t_block = 1;

t_shift = 0;


t_start = (start_time : t_block : end_time) + t_shift;
t_end = start_time + t_block + t_shift : t_block : end_time;

dt = 1/140; % approximate servo rate

for i = 1 : min(length(t_start), length(t_end))
  airspeed_dat{i} = BuildIdDataRPYAirspeed(est, airspeed_unchecked, u, t_start(i), t_end(i), dt, delay_ms);
  
  
end

%if use_airspeed
    dat = airspeed_dat;
   
%end
%   
% for i = 57 : length(airspeed_dat)
%   plot(airspeed_dat{i})
%   title(i)
%   drawnow
%   pause
% end


%merge_nums = [1, 2, 3, 4];
%merge_nums = [1, 2, 3];
%merge_nums = [100, 150, 200];

% interesting data: 4, 8, 9, 16, 20
% aileron roll at about data = 63, 65
merge_nums = [8, 63, 100];



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

x0_dat_full = FixInitialConditionsForData(merged_airspeed_dat);
  


%initial_states = { [0 0] [0 0] [0 0] [0 0] [0 0] [0 0] [10 10] [0 0] [0 0] [0 0] [0 0] [0 0] };


%parameters = [1.92; 1.84; 2.41; 0.48; 0.57; 0.0363];
parameters = [1; 1; 1; 1; 1];

nlgr = idnlgrey(file_name, order, parameters, x0_dat_full);

nlgr.Parameters(1).Name = 'Jx';
nlgr.Parameters(2).Name = 'Jy';
nlgr.Parameters(3).Name = 'Jz';
nlgr.Parameters(4).Name = 'Elift';
nlgr.Parameters(5).Name = 'Edrag';
%nlgr.Parameters(6).Name = 'M_Q_fac';
% nlgr.Parameters(7).Name = 'By_dr';
% nlgr.Parameters(8).Name = 'Bz_dr';

nlgr.Parameters(1).Minimum = 0;
nlgr.Parameters(2).Minimum = 0;
nlgr.Parameters(3).Minimum = 0;
nlgr.Parameters(4).Minimum = 0;
nlgr.Parameters(5).Minimum = 0;
%nlgr.Parameters(6).Minimum = -5;
% nlgr.Parameters(7).Minimum = 0;
% nlgr.Parameters(8).Minimum = 0;

nlgr.Parameters(1).Maximum = 5;
nlgr.Parameters(2).Maximum = 5;
nlgr.Parameters(3).Maximum = 5;
nlgr.Parameters(4).Maximum = 5;
nlgr.Parameters(5).Maximum = 5;
%nlgr.Parameters(6).Maximum = 5;
% nlgr.Parameters(7).Maximum = 0.5;
% nlgr.Parameters(8).Maximum = 0.5;


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


nlgr.Algorithm.Regularization.Lambda = 0.01; % use regularization
nlgr.Algorithm.Regularization.Nominal = 'model'; % attempt to keep parameters close to initial guesses

num_experiments = length(merge_nums);
num_states_floating = 5;

RR = diag([0.01 * ones(length(parameters),1); 0.01*ones(num_experiments * num_states_floating, 1)]);

%RR(2,2) = 1000;
%RR(4,4) = 10000;

% add some regularization on yaw since there isn't much data there
RR(3,3) = 5;


nlgr.Algorithm.Regularization.R = RR;


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

% weight the airspeed output less

roll_weight = 1;
pitch_weight = 3;
yaw_weight = 0.75;
airspeed_weight = 0.025;

output_weights = diag([roll_weight, pitch_weight, yaw_weight, airspeed_weight]);

nlgr.Algorithm.Weighting = output_weights;

%% plot data

disp('Plotting data...');
figure(25);
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
disp(' ---------------------------------------');



disp('Simulating...');


% get initial conditions

for i = 1 : num_states
  
  x0_out(i,:) = nlgr_fit.InitialStates(i).Value;
  
end

compare_options = compareOptions('InitialCondition',x0_out);

%[y_out, fit_out, x0_out] = compare(merged_dat, nlgr_fit);
figure(26);
compare(merged_dat, nlgr_fit, compare_options);

disp('done.');

%% compare to other data

%compare_to = [26, 27];

compare_to = [66, 84, 103];

dat_compare = merge(dat{compare_to});


% estimate initial states for comparison data set

disp(['Estimating initial states for ' num2str(compare_to) '...']);

x0_fixed_compare = FixInitialConditionsForData(dat_compare);

for i = 1:length(nlgr_fit.Parameters)
  parameters_compare(i) = nlgr_fit.Parameters(i).Value;
end

nlgr_compare = idnlgrey(file_name, order, parameters_compare, x0_fixed_compare);

nlgr_compare.Algorithm.Display = 'full';
nlgr_compare = setinit(nlgr_compare, 'Fixed', {true true true true true true true false false false false false });
for i = 1:length(nlgr_compare.Parameters)
  nlgr_compare.Parameters(i).Fixed = 1;
end


%x0 = zeros(num_states, length(merge_nums));

x0_compare = findstates(nlgr_compare, dat_compare)


compare_options2 = compareOptions('InitialCondition',x0_compare);



figure(27)
plot(dat_compare)

figure(28)
compare(dat_compare, nlgr_fit, compare_options2);
