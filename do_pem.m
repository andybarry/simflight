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

t_block = 0.5;
%end_time = start_time + t_block;


t_start = start_time : t_block : end_time;
t_end = start_time + t_block : t_block : end_time;

for i = 1:min(length(t_start), length(t_end))
  
  u_array{i} = TrimU(t_start(i), t_end(i), u);
  imu_array{i} = TrimIMU(t_start(i), t_end(i), imu);
  baro_array{i} = TrimBaro(t_start(i), t_end(i), baro);
  battery_array{i} = TrimBattery(t_start(i), t_end(i), battery);
  est_array{i} = TrimEst(t_start(i), t_end(i), est);
  gps_array{i} = TrimGPS(t_start(i), t_end(i), gps);
  stereo_array{i} = TrimStereo(t_start(i), t_end(i), stereo);
  stereo_replay_array{i} = TrimStereo(t_start(i), t_end(i), stereo_replay);
  stereo_octomap_array{i} = TrimStereoOctomap(t_start(i), t_end(i), stereo_octomap);
  wind_gspeed_array{i} = TrimWindGspeed(t_start(i), t_end(i), wind_gspeed);
end

%% setup model inputs

x = zeros(length(est.vel.x), 1);
y = zeros(length(est.vel.x), 1);
z = zeros(length(est.vel.x), 1);

x_s = spline(est.logtime, x);
y_s = spline(est.logtime, y);
z_s = spline(est.logtime, z);

rpy = quat2rpy_array(est.orientation.q0, est.orientation.q1, est.orientation.q2, est.orientation.q3);

roll_s = spline(est.logtime, rpy(:,1));
pitch_s = spline(est.logtime, rpy(:,2));
yaw_s = spline(est.logtime, rpy(:,3));

U = est.vel.x;
V = est.vel.y;
W = est.vel.z;

U_s = spline(est.logtime, U);
V_s = spline(est.logtime, V);
W_s = spline(est.logtime, W);

Q = est.rotation_rate.x;
P = est.rotation_rate.y;
R = est.rotation_rate.z;

Q_s = spline(est.logtime, Q);
P_s = spline(est.logtime, P);
R_s = spline(est.logtime, R);

u.smooth.elevonL = foh(u.logtime', u.elevonL');
u.smooth.elevonR = foh(u.logtime', u.elevonR');
u.smooth.throttle= foh(u.logtime', u.throttle');

x0 = [ x(1); y(1); z(1); rpy(1,1); rpy(1,2); rpy(1,3); U(1); V(1); W(1); Q(1); P(1); R(1) ];


%% resample at constant dt

dt = 1/140; % appoximate actual servo rate

t0 = max(min(est.logtime), min(u.logtime));
tf = min(max(est.logtime), max(u.logtime));

t = t0:dt:tf;


rpy0  = [ ppval(t, roll_s); ppval(t, pitch_s); ppval(t, yaw_s) ]';
u0 = [ ppval(t, u.smooth.elevonL); ppval(t, u.smooth.elevonR); ppval(t, u.smooth.throttle) ]';



%% setup comparison to model output for orientation PEM

dat = iddata(rpy0, u0, dt);

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
compare(dat, nlgr_fit);
