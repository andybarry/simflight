
% load logs

clear
dir = '2015-03-31-field-test/gps-logs/';
filename = 'lcmlog_2015_03_31_11.mat';



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');
loadDeltawing


[t_starts, t_ends] = FindActiveTimes(u.logtime, u.is_autonomous, 0.5);

t_start = t_starts(6);
t_end = t_ends(6);

u = TrimU(t_start, t_end, u);
est = TrimEst(t_start, t_end, est);

%%


yaw_t0 = est.orientation.yaw(1);


est.drake_frame = zeros(length(est.pos.x), 12);
for i = 1 : length(est.pos.x)

  x_drake_frame = ConvertStateEstimatorToDrakeFrame(est.est_frame(i,:)');

  est.drake_frame(i,:) = x_drake_frame;
end





