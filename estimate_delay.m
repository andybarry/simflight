
clear

%% load delay logs

realtime_path = '/home/abarry/realtime/';
logfile_path = '/home/abarry/rlg/logs/2014-12-05-delay-estimation/mat/';

%logfile_name = 'right_elevon_rightside_up.mat';
logfile_name = 'left_elevon_upside_down.mat';

% add log parsing scripts to path

addpath([realtime_path 'scripts/logs']);

% load data
dir = logfile_path;
filename = logfile_name;

loadDeltawing


%% set up data

optotrak_data = optotrak.y;
u_data = u.elevonL;



figure(1)
clf
plotyy(optotrak.logtime, optotrak_data, u.logtime, u_data)
legend('optotrak','u');

% create splines of the data

t_start = 26.8;
t_end = 28.6;

[~, optotrak_idx_start] = min(abs(optotrak.logtime - t_start));
[~, optotrak_idx_end] = min(abs(optotrak.logtime - t_end));

[~, u_idx_start] = min(abs(u.logtime - t_start));
[~, u_idx_end] = min(abs(u.logtime - t_end));

x_s = spline(optotrak.logtime(optotrak_idx_start:optotrak_idx_end), optotrak_data(optotrak_idx_start:optotrak_idx_end));
u_r_s = foh(u.logtime(u_idx_start:u_idx_end)', u_data((u_idx_start:u_idx_end))');

dt = 1/250;
t = t_start:dt:t_end;

x_eval = ppval(x_s, t);
u_r_eval = -ppval(u_r_s, t);

figure(2)
clf
plotyy(t, x_eval, t, u_r_eval);
legend('optotrak','u');

%% normalize vectors

x_norm = x_eval / norm(x_eval);

u_r_zeroed = u_r_eval - mean(u_r_eval);
u_r_norm = u_r_zeroed / norm(u_r_zeroed);

figure(3)
clf
plot(x_norm);
hold all
plot(u_r_norm);
legend('optotrak','u');
grid on

%% estimate delay



delay = finddelay(u_r_norm, x_norm);

delay_sec = delay*dt