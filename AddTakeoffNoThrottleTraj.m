function lib = AddTakeoffNoThrottleTraj(lib, straight_traj)

  assert(strcmp(straight_traj.name, 'TI-straight-R-200'), 'straight traj name doesnt match.');
  
  % use the same trajectory but zap the throttle
  
  traj = straight_traj;
  
  u0 = straight_traj.utraj.eval(0);
  
  D = straight_traj.lqrsys.D.eval(0);
  D(3,:) = zeros(1,12);
  traj.lqrsys.D = ConstantTrajectory(D);
  
  y0 = straight_traj.lqrsys.y0.eval(0);
  y0(3) = 0;
  traj.lqrsys.y0 = ConstantTrajectory(y0);
  
  traj.utraj = ConstantTrajectory([u0(1); u0(2); 0]);
  
  
  
  traj.name = 'takeoff-no-throttle';
  traj.comments = 'Modified straight trajectory for no throttle.';
  
  lib = lib.InsertExistingTrajectory(traj);
  

end