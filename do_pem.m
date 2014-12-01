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
end_time = start_time + 0.2;

assert(length(start_time) == 1, 'Number of active times ~= 1');



u = TrimU(start_time, end_time, u);
imu = TrimIMU(start_time, end_time, imu);
baro = TrimBaro(start_time, end_time, baro);
battery = TrimBattery(start_time, end_time, battery);
est = TrimEst(start_time, end_time, est);
gps = TrimGPS(start_time, end_time, gps);
stereo = TrimStereo(start_time, end_time, stereo);
stereo_replay = TrimStereo(start_time, end_time, stereo_replay);
stereo_octomap = TrimStereoOctomap(start_time, end_time, stereo_octomap);
wind_gspeed = TrimWindGspeed(start_time, end_time, wind_gspeed);

%% setup model inputs

x = zeros(length(est.vel.x), 1);
y = zeros(length(est.vel.x), 1);
z = zeros(length(est.vel.x), 1);

rpy = quat2rpy_array(est.orientation.q0, est.orientation.q1, est.orientation.q2, est.orientation.q3);

roll = rpy(:,1);
pitch = rpy(:,2);
yaw = rpy(:,3);

U = est.vel.x;
V = est.vel.y;
W = est.vel.z;

Q = est.rotation_rate.x;
P = est.rotation_rate.y;
R = est.rotation_rate.z;

x0 = [ x, y, z, roll, pitch, yaw, U, V, W, Q, P, R ];

u0 = [ u.elevonL, u.elevonR, u.throttle ];


%% setup comparison to model output for orientation PEM

dat = iddata(rpy, u0);

file_name = 'tbsc_model_pem_wrapper';

order = [3, 3, 12];

initial_states = zeros(12, 1);

parameters = [1];

nlgr = idnlgrey(file_name, order, parameters, initial_states, 0);

nlgr_fit = pem(dat, nlgr, 'Display', 'Full', 'MaxIter', 100);

%figure;
%compare(dat, nlgr_fit);
