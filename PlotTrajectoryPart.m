function PlotTrajectoryPart(est, u, traj_t, traj_x, traj_u, title_str)

  % plot roll
  
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
  
  
  % plot pitch
  
  label = 'Roll (deg)';
  figure(2)
  
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
  
  
  % plot yaw
  
  label = 'Yaw (deg)';
  figure(3)
  
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
  
  % plot x and y
  
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
  figure(4);
  
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
  figure(5);
  
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
  
  
  
  

end