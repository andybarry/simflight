%% load data

clear
addpath('/home/abarry/realtime/scripts/logs');
%% setup loading variables

date = '2015-10-06';
name = 'field-test';
log_number = '11';
stabilization_trajectory = 0;
hostname = 'odroid-gps3';

trajectory_library = 'traj-archive/oct6-from-data.mat';

use_simulation = false;

%% load trajectory library

disp(['Loading ' trajectory_library '...']);
load(trajectory_library);
disp('done.');

%% load log data

dir = [date '-' name '/' hostname '/'];
filename = ['lcmlog_' strrep(date, '-', '_') '_' log_number '.mat'];


dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');

disp(['Loading ' filename '...']);

loadDeltawing


disp('done.');

%% remove trajectories on the ground

[t_starts_unfiltered, t_ends_unfiltered] = FindActiveTimes(u.logtime, u.is_autonomous, 0.5);

t_starts = [];
t_ends = [];

for i = 1 : length(t_starts_unfiltered)
  [~, idx] = min(abs(est.logtime - t_starts_unfiltered(i)));
  [~, idx_end] = min(abs(est.logtime - t_ends_unfiltered(i)));
  
  this_alt = est.pos.z(idx);
  this_alt_end = est.pos.z(idx_end);
  this_alt_mid = est.pos.z( round((idx_end-idx)/2) + idx );
  this_alt_max = max(est.pos.z);
  
  if (this_alt > 5 || this_alt_end > 5 || this_alt_mid > 7.5 || this_alt_max > 7.5)
    t_starts = [t_starts t_starts_unfiltered(i)];
    t_ends = [t_ends t_ends_unfiltered(i)];
  else
    disp(['t = ' num2str(t_starts_unfiltered(i)) ' to ' num2str(t_ends_unfiltered(i)) ' filtered for being on the ground.']);
  end
end



%% simulate

if use_simulation

    for i = 1:length(t_starts)

      xtrajsim{i} = SimulateTrajectoryAgainstData(u, est, parameters, t_starts(i), t_ends(i));

    end
else
    xtrajsim = {};
end


%% plot

for i = 1:length(t_starts)
  if use_simulation
    TrajectoryToDataComparisonPlotter(u, est, tvlqr_out, lib, t_starts(i), t_ends(i), stabilization_trajectory);
  else
    TrajectoryToDataComparisonPlotter(u, est, tvlqr_out, lib, t_starts(i), t_ends(i), stabilization_trajectory);
  end
  
  pause
  
end

%% create plots for papers/talks
%tmin = 125.4;
tmin = 155;
tmax = 170;

for i = 1 : 5
  figure(i)
  for j = 1 : 3
    subplot(3,1,j)
    xlim([tmin tmax]);
  end
end

%% save files

name_str = '2015-10-08_10';
disp(['Saving ' name_str '...']);

SaveComparison([name_str '-x'], 1, 1);
SaveComparison([name_str '-y'], 1, 2);
SaveComparison([name_str '-z'], 1, 3);

SaveComparison([name_str '-roll'], 2, 1);
SaveComparison([name_str '-pitch'], 2, 2);
SaveComparison([name_str '-yaw'], 2, 3);

SaveComparison([name_str '-u-left'], 3, 1);
SaveComparison([name_str '-u-right'], 3, 2);

disp('done.');