function data = BuildIdDataRPY(est, u, t_start, t_end, dt)
  % Builds iddata from sensor values
  
  u = TrimU(t_start, t_end, u);
  est = TrimEst(t_start, t_end, est);
  

  rpy = quat2rpy_array(est.orientation.q0, est.orientation.q1, est.orientation.q2, est.orientation.q3);

  roll_s = spline(est.logtime, rpy(:,1));
  pitch_s = spline(est.logtime, rpy(:,2));
  yaw_s = spline(est.logtime, rpy(:,3));
  
  
  u.smooth.elevonL = foh(u.logtime', u.elevonL');
  u.smooth.elevonR = foh(u.logtime', u.elevonR');
  u.smooth.throttle= foh(u.logtime', u.throttle');
  
  t0 = max(min(est.logtime), min(u.logtime));
  tf = min(max(est.logtime), max(u.logtime));

  t = t0:dt:tf;
  
  rpy0  = [ ppval(t, roll_s); ppval(t, pitch_s); ppval(t, yaw_s) ]';
  u0 = [ ppval(t, u.smooth.elevonL); ppval(t, u.smooth.elevonR); ppval(t, u.smooth.throttle) ]';

  data = iddata(rpy0, u0, dt);

end