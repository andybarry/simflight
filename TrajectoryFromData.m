%% load data

clear
%%

date = '2015-09-17';
name = 'field-test';
log_number = '04';
hostname = 'odroid-gps2';


dir = [date '-' name '/' hostname '/'];
filename = ['lcmlog_' strrep(date, '-', '_') '_' log_number '.mat'];


dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');

disp(['Loading ' filename '...']);

loadDeltawing

disp('done.');

%% extract time

start_t = 127.629;
end_t = 131.0;

u_trim = TrimU(start_t, end_t, u);
est_trim = TrimEst(start_t, end_t, est);

%% create xtraj
est_t0 = est_trim.est_frame(1,:);

% zero yaw
est_frame = est_trim.est_frame;

rpy_to_zero = [0 0 -est_t0(6)];
Mz = rpy2rotmat(rpy_to_zero);

drake_frame = zeros(size(est_frame));
for i = 1 : size(est_frame, 1)
  drake_frame(i, :) = ConvertStateEstimatorToDrakeFrame(est_frame(i, :)', Mz);
end

drake_t0 = drake_frame(1,:);

% zero x, y and z
drake_frame(:, 1:3) = drake_frame(:, 1:3) - repmat(drake_t0(1:3), size(drake_frame, 1), 1);

xtraj = PPTrajectory(spline(est_trim.logtime - est_trim.logtime(1), drake_frame'));

%% create utraj


utraj = PPTrajectory(spline(u_trim.logtime - u_trim.logtime(1), u_trim.rad.zeroed_vector'));

%%
traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, state_frame, 'test', 'test');

