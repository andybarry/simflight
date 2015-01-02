function xtraj = runInputTape(p, utraj, tf)


  x = 0;
  y = 0;
  z = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
  xdot = 13;
  ydot = 0;
  zdot = 0;
  rolldot = 0;
  pitchdot = 0;
  yawdot = 0;

  x0 = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ];

  sys = cascade(utraj, p);

  disp('Simulating...');
  xtraj = simulate(sys, [0 tf], x0);
  disp('done.');
  
  DrawTrajectoryLcmGl(xtraj, 'trajectory', struct('sphere_size', 0.01));

  p.playback(xtraj, utraj, struct('slider', true));


end