%function xtraj = runTvlqr(p, lqr_controller)


  x = 0;
  y = 0;
  z = 0;
  roll = 0;
  pitch = 0;
  yaw = 0.3;
  xdot = 13;
  ydot = 0;
  zdot = 0;
  rolldot = 0;
  pitchdot = 0;
  yawdot = 0;
  
  tf = 0.5;

  x0 = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ];
  
  feedback_sys = feedback(p, lqr_controller);
  
  
  disp('Simulating with TVLQR controller...');
  [ytraj, xtraj_tvlqr] = simulate(feedback_sys, [0 tf], x0);
  disp('done.');
  
  clear u;
  
  % compute the utraj from the xtraj
  breaks = xtraj_tvlqr.getBreaks();
  for i = 1 : length(breaks)
    t = breaks(i);
    
    u(i,:) = lqr_controller.output(t, [], xtraj_tvlqr.eval(t) - xtraj.eval(t));
  end
  
  u_spline = spline(breaks, u' + utraj.eval(xtraj_tvlqr.getBreaks()));
  
  utraj_tvlqr = PPTrajectory(u_spline);
  
  DrawTrajectoryLcmGl(xtraj, 'trajectory', struct('sphere_size', 0.01,'color',[0 1 0]));
  
  DrawTrajectoryLcmGl(xtraj_tvlqr, 'tvlqr_trajectory', struct('sphere_size', 0.01, 'color',[1 0 0]));

  %p.playback_xtraj(xtraj, struct('slider', true));
  
  p.playback(xtraj_tvlqr, utraj_tvlqr, struct('slider', true));


%end