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

% hack?
u0 = u0(2:end, :);


%% setup comparison to model output

Udot = est.accel.x;
Vdot = est.accel.y;
Wdot = est.accel.z;

Pdot = diff(est.rotation_rate.x);
Qdot = diff(est.rotation_rate.y);
Rdot = diff(est.rotation_rate.z);

% hack?
x0 = x0(2:end, :);
u0 = u0(2:end, :);
Udot = Udot(2:end);
Vdot = Vdot(2:end);
Wdot = Wdot(2:end);

xdot_compare_measured = [ Udot Vdot, Wdot, Pdot, Qdot, Rdot ];

%% setup nonlinear least squares to fit accelerations and angular rates

model_wrapper = @(elev_drag_fac) tbsc_model_wrapper(x0, u0, xdot_compare_measured, elev_drag_fac);


[ params, resnorm ] = lsqnonlin(model_wrapper, 1, 0, 100)

