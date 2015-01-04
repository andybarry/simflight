%function xtraj = runTvlqr(p, lqr_controller)


  x = 0;
  y = 0;
  z = 0;
  roll = 0.2;
  pitch = 0;
  yaw = 0;
  xdot = 13;
  ydot = 0;
  zdot = 0;
  rolldot = 0;
  pitchdot = 0;
  yawdot = 0;
  
  tf = 0.5;

  x0 = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ];
  
  feedback_sys = feedback(p, lqr_controller);
  
  
  disp('Simulating...');
  [ytraj, xtraj] = simulate(feedback_sys, [0 tf], x0);
  disp('done.');
  
  % compute the utraj from the xtraj
  breaks = xtraj.getBreaks();
  for i = 1 : length(breaks)
    t = breaks(i);
    
    u(i,:) = lqr_controller.output(t, [], xtraj.eval(t));
  end
  
  u_spline = spline(breaks, u');
  
  utraj = PPTrajectory(u_spline);
  
  
  DrawTrajectoryLcmGl(xtraj, 'trajectory', struct('sphere_size', 0.01));

  %p.playback_xtraj(xtraj, struct('slider', true));
  
  p.playback(xtraj, utraj, struct('slider', true));


%end