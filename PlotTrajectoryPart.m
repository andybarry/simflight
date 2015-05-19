function PlotTrajectoryPart(est, u, traj_t, traj_x, traj_u, title_str)

  figure_num = 1;

  %% plot roll
  
  label = 'Roll (deg)';
  figure(1)
  
  plot(est.logtime, rad2deg(est.orientation.roll), 'b-');
  hold on
  
  plot(traj_t, rad2deg(traj_x(4,:)), 'r-');
  
  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  
  %% plot pitch
  
  
  label = 'Roll (deg)';
  figure_num = figure_num + 1;
  figure(figure_num)
  
  plot(est.logtime, rad2deg(est.orientation.pitch), 'b-');
  hold on
  
  plot(traj_t, rad2deg(traj_x(5,:)), 'r-');

  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  
  %% plot yaw
  
  label = 'Yaw (deg)';
  figure_num = figure_num + 1;
  figure(figure_num)
  
  plot(est.logtime, rad2deg(est.orientation.yaw), 'b-');
  hold on
  
  plot(traj_t, rad2deg(traj_x(6,:) + est.orientation.yaw(1)), 'r-');

  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  %% plot x and y
  
  % first, compute the mulipliers on x and y that come from the fact that
  % we didn't start heading exactly forward
  yaw0 = est.orientation.yaw(1);

  R = [ cos(yaw0) -sin(yaw0);
      sin(yaw0) cos(yaw0)];
    
   for i = 1 : length(traj_t)

    this_point = traj_x(1:2, i);
    traj_points_local(i, :) = R * this_point;
  end
  
  label = 'X (m);';
  figure_num = figure_num + 1;
  figure(figure_num)
  
  plot(est.logtime, est.pos.x, 'b-');
  hold on
  plot(traj_t, traj_points_local(:,1) + est.pos.x(1), 'r-');
  
  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  label = 'Y (m);';
  figure_num = figure_num + 1;
  figure(figure_num)
  
  plot(est.logtime, est.pos.y, 'b-');
  hold on
  plot(traj_t, traj_points_local(:,2) + est.pos.y(1), 'r-');
  
  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  %% plot z
  
  label = 'Z (ft)';
  figure_num = figure_num + 1;
  figure(figure_num)
  
  plot(est.logtime, est.pos.z, 'b-');
  hold on
  
  plot(traj_t, traj_x(3,:) + est.pos.z(1), 'r-');

  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  %% plot u
  
  label = 'Control surface deflection (deg)';
  figure_num = figure_num + 1;
  figure(figure_num)
  
  subplot(3,1,1);
  plot(u.logtime, rad2deg(u.rad.elevonL), 'b-');
  hold on
  plot(traj_t, rad2deg(traj_u(1,:)), 'b--');
  grid on
  xlabel('Time (s)');
  ylabel(label);
  title([ 'LEFT: ' label ': ' title_str]);
  legend('Actual', 'Planned')
  
  subplot(3,1,2);
  plot(u.logtime, rad2deg(u.rad.elevonR), 'r-');
  hold on
  plot(traj_t, rad2deg(traj_u(2, :)), 'r--');
  

  grid on
  xlabel('Time (s)');
  ylabel(label);
  title([ 'RIGHT: ' label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  subplot(3,1,3);
  plot(u.logtime, u.rad.throttle, 'r-');
  hold on
  plot(traj_t, traj_u(3, :), 'r--');
  

  grid on
  xlabel('Time (s)');
  ylabel(label);
  title([ 'THROTTLE: ' label ': ' title_str]);
  set(gca, 'XLimMode', 'auto');
  set(gca, 'YLimMode', 'auto');
  
  
  
  
  

end