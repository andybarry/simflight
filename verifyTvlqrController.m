
% load logs

%clear

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
%%
u = TrimU(t_start, t_end, u);
est = TrimEst(t_start, t_end, est);

%%

init_state = ConvertStateEstimatorToDrakeFrame(est.est_frame(1,:));
rpy = init_state(4:6);
Mz = rotz(-rpy(3));

x0 = traj.xtraj.eval(0);
u0 = traj.utraj.eval(0);

K = traj.lqrsys.D.eval(0);

est.drake_frame = zeros(length(u.logtime), 12);
for i = 1 : length(u.logtime)
  
  [~, idx] = min(abs(u.logtime(i) - est.logtime));
  %idx = i;
  
  this_x = est.est_frame(idx, :);
  
  x_drake_frame = ConvertStateEstimatorToDrakeFrame(this_x, Mz);

  est.drake_frame(i,:) = x_drake_frame;
  
  error = x_drake_frame - x0;

  u_out(:,i) = u0 + K * error;
  
  % compute percentages of contribution for each state
  delta_u = K * error;
  
  for state_num = 1 : 12
    
    this_contrib = (K(:,state_num) * error(state_num) );
    
    contrib_elevL(state_num, i) = this_contrib(1);
    contrib_elevR(state_num, i) = this_contrib(2);
    contrib_throttle(state_num, i) = this_contrib(3);
    
  end
  
end

% convert to servo commands
u_out_servo(1,:) = round(u_out(1,:) * rad_to_servo.elevL_slope + rad_to_servo.elevL_y_intercept);
u_out_servo(2,:) = round(u_out(2,:) * rad_to_servo.elevR_slope + rad_to_servo.elevR_y_intercept);
u_out_servo(3,:) = round(u_out(3,:) * rad_to_servo.throttle_slope + rad_to_servo.throttle_y_intercept);

figure(1)
clf
plot(u.logtime, u.elevonL)
hold on
plot(u.logtime, u_out_servo(1,:)','r');

legend('Actual','Replay');



figure(2)
clf
plot(u.logtime, u.elevonR)
hold on
plot(u.logtime, u_out_servo(2,:)','r');

legend('Actual','Replay');


figure(3)
clf
plot(u.logtime, u.throttle)
hold on
plot(u.logtime, u_out_servo(3,:)','r');

legend('Actual','Replay');

figure(4)
clf
plot(u.logtime, rad2deg(contrib_elevL(4, :)'), 'b');
hold on
plot(u.logtime, rad2deg(contrib_elevL(5, :)'), 'r');
plot(u.logtime, rad2deg(contrib_elevL(6, :)'), 'k');
plot(u.logtime, rad2deg(contrib_elevL(7, :)'), '--b');
plot(u.logtime, rad2deg(contrib_elevL(8, :)'), '--r');
plot(u.logtime, rad2deg(contrib_elevL(9, :)'), '--k');
plot(u.logtime, rad2deg(contrib_elevL(10, :)'), 'g');
plot(u.logtime, rad2deg(contrib_elevL(11, :)'), '--g');
plot(u.logtime, rad2deg(contrib_elevL(12, :)'), '-.b');
xlabel('Time (s)');
ylabel('Deg deflection');
%legend('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
legend('roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');


figure(5)
clf
plot(u.logtime, rad2deg(contrib_elevR(4, :)'), 'b');
hold on
plot(u.logtime, rad2deg(contrib_elevR(5, :)'), 'r');
plot(u.logtime, rad2deg(contrib_elevR(6, :)'), 'k');
plot(u.logtime, rad2deg(contrib_elevR(7, :)'), '--b');
plot(u.logtime, rad2deg(contrib_elevR(8, :)'), '--r');
plot(u.logtime, rad2deg(contrib_elevR(9, :)'), '--k');
plot(u.logtime, rad2deg(contrib_elevR(10, :)'), 'g');
plot(u.logtime, rad2deg(contrib_elevR(11, :)'), '--g');
plot(u.logtime, rad2deg(contrib_elevR(12, :)'), '-.b');
xlabel('Time (s)');
ylabel('Deg deflection');
%legend('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
legend('roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
