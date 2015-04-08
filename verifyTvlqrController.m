
% load logs

clear

FindTrimDrake


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


x_t0 = est.pos.x(1);
y_t0 = est.pos.y(1);
z_t0 = est.pos.z(1);
yaw_t0 = est.orientation.yaw(1);

x0 = traj.xtraj.eval(0);
u0 = traj.utraj.eval(0);

K = traj.lqrsys.D.eval(0);

est.drake_frame = zeros(length(est.pos.x), 12);
for i = 1 : length(est.pos.x)
  this_x = est.est_frame(i,:)';
  
  this_x(1) = this_x(1) - x_t0;
  this_x(2) = this_x(2) - y_t0;
  this_x(3) = this_x(3) - z_t0;
  this_x(6) = this_x(6) - yaw_t0;
  x_drake_frame = ConvertStateEstimatorToDrakeFrame(this_x);

  est.drake_frame(i,:) = x_drake_frame;
  
  error = x_drake_frame - x0;

  u_out(:,i) = u0 + K * error;
  
end

% convert to servo commands
u_out_servo(1,:) = round(u_out(1,:) * rad_to_servo.elevL_slope + rad_to_servo.elevL_y_intercept);
u_out_servo(2,:) = round(u_out(2,:) * rad_to_servo.elevR_slope + rad_to_servo.elevR_y_intercept);
u_out_servo(3,:) = round(u_out(3,:) * rad_to_servo.throttle_slope + rad_to_servo.throttle_y_intercept);

figure(1)
clf
plot(u.logtime, u.elevonL)
hold on
plot(est.logtime, u_out_servo(1,:)','r');

legend('Actual','Replay');



figure(2)
clf
plot(u.logtime, u.elevonR)
hold on
plot(est.logtime, u_out_servo(2,:)','r');

legend('Actual','Replay');
