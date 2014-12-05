function data = BuildIdDataRPYAirspeed(est, baro, u, t_start, t_end, dt)
  % Builds iddata from sensor values
  
  u = TrimU(t_start, t_end, u);
  est = TrimEst(t_start, t_end, est);
  baro = TrimBaro(t_start, t_end, baro);
  

  rpy = quat2rpy_array(est.orientation.q0, est.orientation.q1, est.orientation.q2, est.orientation.q3);

  rpy_body = zeros(size(rpy));
  
  for i = 1 : size(rpy, 1)
    x_world = [ zeros(3,1); rpy(i,:)'; zeros(6,1)];
    x_body = ConvertToModelFrameFromDrakeWorldFrame(x_world);
    rpy_body(i,:) = x_body(4:6);
  end
  
  roll_s = spline(est.logtime, rpy_body(:,1));
  pitch_s = spline(est.logtime, rpy_body(:,2));
  yaw_s = spline(est.logtime, rpy_body(:,3));
  
  airspeed_s = spline(baro.logtime, baro.airspeed);
  
  
  u.smooth.elevonL = foh(u.logtime', u.elevonL');
  u.smooth.elevonR = foh(u.logtime', u.elevonR');
  u.smooth.throttle= foh(u.logtime', u.throttle');
  
  t0 = max(min(est.logtime), min(min(u.logtime), min(baro.logtime)));
  tf = min(max(est.logtime), max(max(u.logtime), max(baro.logtime)));

  t = t0:dt:tf;
  
  rpy_airspeed0  = [ ppval(t, roll_s); ppval(t, pitch_s); ppval(t, yaw_s); ppval(t, airspeed_s)]';
  u0 = [ ppval(t, u.smooth.elevonL); ppval(t, u.smooth.elevonR); ppval(t, u.smooth.throttle) ]';

  data = iddata(rpy_airspeed0, u0, dt);
  
  set(data, 'InputName',{'elevL', 'elevR', 'throttle'}, 'OutputName',{'roll','pitch','yaw', 'airspeed'})

end