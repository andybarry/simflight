
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


%% extract initial conditions

rpy = quat2rpy_array(est.orientation.q0, est.orientation.q1, est.orientation.q2, est.orientation.q3);

x = 0;
y = 0;
z = 0;
roll = rpy(1,1);
pitch = rpy(1,2);
yaw = rpy(1,3);
xdot = baro.airspeed(1);
ydot = 0;
zdot = 0;
rolldot = 0; % todo?
pitchdot = 0;
yawdot = 0;

x0_drake = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ]

%% build a trajectory for the inputs

dt = 1/140;

u_pp = foh(u.logtime', [u.elevonL u.elevonR u.throttle]');

utraj = PPTrajectory(u_pp);

%% build drake objects

p = DeltawingPlant();

utraj = utraj.setOutputFrame(p.getInputFrame());

options.floating = true;
r = RigidBodyManipulator('TBSC_visualizer.urdf', options);

v2 = HudBotVisualizer(r);
%v2 = r.constructVisualizer();



feedback_system = cascade(utraj, p);

traj = simulate(feedback_system, utraj.tspan, x0_drake)

%% visualize

% combine the simulated trajectory with the inputs

traj_and_u = [traj; utraj];

fr = traj_and_u.getOutputFrame();

transform_func = @(t, x, x_and_u) [ x_and_u(1:6); utraj(3); utraj(1:2); x_and_u(7:12); zeros(3,1)];

trans = FunctionHandleCoordinateTransform(17, 0, traj_and_u.getOutputFrame(), v2.getInputFrame(), true, true, transform_func, transform_func, transform_func);

fr.addTransform(trans);


playback(v2, traj_and_u, struct('slider', true));


