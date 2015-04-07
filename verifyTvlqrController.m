
% load logs

%clear
dir = '2015-03-31-field-test/gps-logs/';
filename = 'lcmlog_2015_03_31_11.mat';



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

loadDeltawing


[t_starts, t_ends] = FindActiveTimes(u.logtime, u.is_autonomous, 0.5);

t_start = t_starts(6);
t_end = t_ends(6);


% build state for controller

[~, index_t0] = min(abs(u.logtime - t_start));


quat_t0 = [est.orientation.q0(index_t0) est.orientation.q1(index_t0) est.orientation.q2(index_t0) est.orientation.q3(index_t0)];

rpy_t0 = quat2rpy(quat_t0);

pos_t0 = [ est.pos.x(index_t0); est.pos.y(index_t0); est.pos.z(index_t0) ];







controller_state = ConvertStateEstimatorToDrakeFrame(x_est_frame);