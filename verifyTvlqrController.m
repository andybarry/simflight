
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% TO USE THIS SCRIPT, LOAD DATA WITH CompareTrajectoriesToData.m %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%t_start_approx = 125.6;
t_start_approx = 121.166;

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

%%

init_state = ConvertStateEstimatorToDrakeFrame(est_trim.est_frame(1,:)');
rpy = init_state(4:6);
Mz = rotz(-rpy(3));

x0 = traj.xtraj.eval(0); % TODO FIXME SHOULD BE Time varying!
u0 = traj.utraj.eval(0); % TODO FIXME SHOULD BE Time varying!

K = traj.lqrsys.D.eval(0); % TODO FIXME SHOULD BE Time varying!

est_trim.drake_frame = zeros(length(u_trim.logtime), 12);

clear u_out_servo contrib_elevL contrib_elevR u_out
for i = 1 : length(u_trim.logtime)

  [~, idx] = min(abs(u_trim.logtime(i) - est_trim.logtime));
  %idx = i;

  
  x_est_frame_converted = PoseToStateEstimatorVector(est_trim, idx, init_state(1:3), Mz);
  
  error = x_est_frame_converted - x0;

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
plot(u_trim.logtime, u_trim.elevonL)
hold on
plot(u_trim.logtime, u_out_servo(1,:)','r');
xlabel('Time (s)');
ylabel('Pulse (us)');
legend('Actual','Replay');
title('Left elevon');


figure(2)
clf
plot(u_trim.logtime, u_trim.elevonR)
hold on
plot(u_trim.logtime, u_out_servo(2,:)','r');
xlabel('Time (s)');
ylabel('Pulse (us)');
legend('Actual','Replay');
title('Right elevon');


figure(3)
clf
plot(u_trim.logtime, u_trim.throttle)
hold on
plot(u_trim.logtime, u_out_servo(3,:)','r');
xlabel('Time (s)');
ylabel('Pulse (us)');
legend('Actual','Replay');
title('Throttle');

figure(4)
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


figure(5)
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
