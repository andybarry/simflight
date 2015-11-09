
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% TO USE THIS SCRIPT, LOAD DATA WITH CompareTrajectoriesToData.m %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


t_start_approx = 125.6;
%t_start_approx = 121.166;
%t_start_approx = 127.34;

% find the nearest trajectory start

[error_t, idx] = min(abs(tvlqr_out.logtime - t_start_approx));

if error_t > 0.1
  error(['No trajectory found near t_start_approx (' num2str(t_start_approx) ').  Nearest trajectory start is ' num2str(tvlqr_out.logtime(idx))]);
end

t_start = tvlqr_out.logtime(idx);
t_end = tvlqr_out.logtime(idx+1);

traj_num = tvlqr_out.trajectory_number(idx);
traj = lib.GetTrajectoryByNumber(traj_num);

disp(['Analyzing trajectory #' num2str(tvlqr_out.trajectory_number(idx)) ' from ' num2str(t_start) ' to ' num2str(t_end)]);
disp(['Traj #' num2str(traj_num) ': ' traj.name]);

%%

u_trim = TrimU(t_start, t_end, u);
est_trim = TrimEst(t_start, t_end, est);
state_init_complete_trim = TrimStateInitComplete(t_start, t_end, state_init_complete);

%%

init_state = ConvertStateEstimatorToDrakeFrame(est_trim.est_frame(1,:)');
t0 = est_trim.logtime(1);
rpy = init_state(4:6);
Mz = rotz(-rpy(3));

est_trim.drake_frame = zeros(length(u_trim.logtime), 12);

% find first state_init_complete message
[~, idx_init_complete] = min(abs(state_init_complete_trim.logtime - t0));

t_state_init_complete = state_init_complete_trim.logtime(idx_init_complete);

delay_for_init = t_state_init_complete - t0;

assert(abs(delay_for_init) < 1e-4, 'State estimator init delay is large, needs to be compensated for');


clear u_out_servo contrib_elevL contrib_elevR u_out
for i = 1 : length(u_trim.logtime)
  
  [~, idx] = min(abs(u_trim.logtime(i) - est_trim.logtime));
  %idx = i;

  traj_t = est_trim.logtime(idx) - t0;
  x0 = traj.xtraj.eval(traj_t);
  this_u0 = traj.utraj.eval(traj_t);
  K = traj.lqrsys.D.eval(traj_t);
  
  
  x_est_frame_converted = PoseToStateEstimatorVector(est_trim, idx, init_state(1:3), Mz);
  
  error_val = x_est_frame_converted - x0;

  u_out(:,i) = this_u0 + K * error_val;
  
  u0(:,i) = this_u0;
  
  % compute percentages of contribution for each state
  delta_u = K * error_val;
  
  for state_num = 1 : 12
    
    this_contrib = (K(:,state_num) * error_val(state_num) );
    
    contrib_elevL(state_num, i) = this_contrib(1);
    contrib_elevR(state_num, i) = this_contrib(2);
    contrib_throttle(state_num, i) = this_contrib(3);
    
  end
  
end

% convert to servo commands
u_out_servo(1,:) = round(u_out(1,:) * rad_to_servo.elevL_slope + rad_to_servo.elevL_y_intercept);
u_out_servo(2,:) = round(u_out(2,:) * rad_to_servo.elevR_slope + rad_to_servo.elevR_y_intercept);
u_out_servo(3,:) = round(u_out(3,:) * rad_to_servo.throttle_slope + rad_to_servo.throttle_y_intercept);

u0_servo(1,:) = round(u0(1,:) * rad_to_servo.elevL_slope + rad_to_servo.elevL_y_intercept);
u0_servo(2,:) = round(u0(2,:) * rad_to_servo.elevR_slope + rad_to_servo.elevR_y_intercept);
u0_servo(3,:) = round(u0(3,:) * rad_to_servo.throttle_slope + rad_to_servo.throttle_y_intercept);

figure(11)
clf
plot(u_trim.logtime, u_trim.elevonL)
hold on
plot(u_trim.logtime, u_out_servo(1,:)','r');
plot(u_trim.logtime, u0_servo(1,:), 'r--');
plot([u_trim.logtime(1) u_trim.logtime(end)], [u.trim.zero.elevonL u.trim.zero.elevonL], 'k--');
xlabel('Time (s)');
ylabel('Pulse (us)');
legend('Actual','Replay','u0');
title('Left elevon');


figure(12)
clf
plot(u_trim.logtime, u_trim.elevonR)
hold on
plot(u_trim.logtime, u_out_servo(2,:)','r');
plot(u_trim.logtime, u0_servo(2,:), 'r--');
plot([u_trim.logtime(1) u_trim.logtime(end)], [u.trim.zero.elevonR u.trim.zero.elevonR], 'k--');
xlabel('Time (s)');
ylabel('Pulse (us)');
legend('Actual','Replay','u0');
title('Right elevon');


figure(13)
clf
plot(u_trim.logtime, u_trim.throttle)
hold on
plot(u_trim.logtime, u_out_servo(3,:)','r');
plot(u_trim.logtime, u0_servo(3,:), 'r--');
xlabel('Time (s)');
ylabel('Pulse (us)');
legend('Actual','Replay','u0');
title('Throttle');

figure(14)
clf
plot(u_trim.logtime, rad2deg(contrib_elevL(4, :)'), 'b');
hold on
plot(u_trim.logtime, rad2deg(contrib_elevL(5, :)'), 'r');
plot(u_trim.logtime, rad2deg(contrib_elevL(6, :)'), 'k');
plot(u_trim.logtime, rad2deg(contrib_elevL(7, :)'), '--b');
plot(u_trim.logtime, rad2deg(contrib_elevL(8, :)'), '--r');
plot(u_trim.logtime, rad2deg(contrib_elevL(9, :)'), '--k');
plot(u_trim.logtime, rad2deg(contrib_elevL(10, :)'), 'g');
plot(u_trim.logtime, rad2deg(contrib_elevL(11, :)'), '--g');
plot(u_trim.logtime, rad2deg(contrib_elevL(12, :)'), '-.b');
xlabel('Time (s)');
ylabel('Deg deflection');
%legend('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
legend('roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
title('Contributions for elevL');


figure(15)
clf
plot(u_trim.logtime, rad2deg(contrib_elevR(4, :)'), 'b');
hold on
plot(u_trim.logtime, rad2deg(contrib_elevR(5, :)'), 'r');
plot(u_trim.logtime, rad2deg(contrib_elevR(6, :)'), 'k');
plot(u_trim.logtime, rad2deg(contrib_elevR(7, :)'), '--b');
plot(u_trim.logtime, rad2deg(contrib_elevR(8, :)'), '--r');
plot(u_trim.logtime, rad2deg(contrib_elevR(9, :)'), '--k');
plot(u_trim.logtime, rad2deg(contrib_elevR(10, :)'), 'g');
plot(u_trim.logtime, rad2deg(contrib_elevR(11, :)'), '--g');
plot(u_trim.logtime, rad2deg(contrib_elevR(12, :)'), '-.b');
xlabel('Time (s)');
ylabel('Deg deflection');
%legend('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
legend('roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot');
title('Contributions for elevR');