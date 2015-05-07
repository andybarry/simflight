function TrajectoryToDataComparisonPlotter(u, est, tvlqr_out, lib, xtrajsim, t_start, t_end)


  [~, idx] = min(abs(tvlqr_out.logtime - t_start));
  this_traj_num = tvlqr_out.trajectory_number(idx);

  traj = lib.GetTrajectoryByNumber(this_traj_num);
  
  disp(['t: ' num2str(t_start) ' -- ' num2str(t_end)]);
  disp(['Trajectory #' num2str(this_traj_num)  ' (' traj.name ')']);

  title_str = ['Trajectory #' num2str(this_traj_num) ' (' traj.name ')'];
  
  xlim_t_extra = 1.5;

  %%
  u = TrimU(t_start, t_end, u);
  est = TrimEst(t_start, t_end, est);


  %%
  dt = 1/140;
  
  if (traj.xtraj.tspan(2) ~= Inf)
    traj_end_t = traj.xtraj.tspan(2);
  else
    traj_end_t = u.logtime(end) - u.logtime(1);
  end
  
  t = 0:dt:traj_end_t;
  trajx = traj.xtraj.eval(t);
  traju = traj.utraj.eval(t);
  trajsim = xtrajsim.eval(t+est.logtime(1));

  %% plot roll

  figure(1)
  clf

  plot(est.logtime, rad2deg(est.orientation.roll), '-r')
  hold on

  traj_roll = trajx(4,:);
  plot(t+est.logtime(1), rad2deg(traj_roll), 'b-')

  plot(t+est.logtime(1), rad2deg(trajsim(4,:)), '--k');

  xlim([t(1)+est.logtime(1) t(end)+est.logtime(1) + xlim_t_extra]);

  grid on
  xlabel('Time (s)');
  ylabel('Roll (deg)');
  legend('Actual', 'Planned','Simulated')
  title([ title_str ', Roll']);

  %% plot pitch

  figure(2)
  clf

  plot(est.logtime, rad2deg(est.orientation.pitch), '-r')
  hold on

  plot(t+est.logtime(1), rad2deg(trajx(5,:)), 'b-')
  plot(t+est.logtime(1), rad2deg(trajsim(5,:)), '--k');

  xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)  + xlim_t_extra]);
  set(gca,'YDir','reverse');
  grid on
  xlabel('Time (s)');
  ylabel('Pitch (deg)');
  legend('Actual', 'Planned','Simulated')
  title([ title_str ', Pitch']);

  %% plot z

  figure(3)
  clf

  plot(est.logtime, est.pos.z, 'r-');
  hold on

  plot(t+est.logtime(1), trajx(3,:) + est.pos.z(1), 'b-');
  plot(t+est.logtime(1), trajsim(3,:), '--k');

  xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)  + xlim_t_extra]);


  grid on
  xlabel('Time (s)');
  ylabel('Altitude (m)');
  legend('Actual', 'Planned','Simulated')
  title([ title_str ', Altitude']);

  %% plot u

  figure(4)
  clf
  plot(u.logtime, u.rad.elevonL);
  hold on
  plot(t+u.logtime(1), traju(1,:),'b--')



  plot(u.logtime, u.rad.elevonR, 'm-');
  plot(t+u.logtime(1), traju(2,:),'m--')
  %plot(u.logtime, u.throttle, 'k-');
  legend('elevonL', 'elevonL-plan', 'elevonR', 'elevonR-plan');
  grid on

  xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)  + xlim_t_extra]);
  title([ title_str ', u']);


end