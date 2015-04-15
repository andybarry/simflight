function data = BuildIdDataRPYAirspeed(est, airspeed, u, t_start, t_end, dt, delay_ms)
  % Builds iddata from sensor values
  
  u = TrimU(t_start, t_end, u);
  est = TrimEst(t_start, t_end, est);
  airspeed = TrimAirspeed(t_start, t_end, airspeed);
  
  delay_sec = delay_ms / 1000;
  

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
  
  [b,a] = butter(1, 0.2);
  airspeed.filtered = filtfilt(b,a,airspeed.airspeed);
  
%   figure(1)
%   clf
%   plot(airspeed.logtime, airspeed.airspeed);
%   hold on
%   plot(airspeed.logtime, airspeed.filtered, 'r-');
  
  airspeed_s = spline(airspeed.logtime, airspeed.filtered);
  
  
  u.smooth.elevonL = foh(u.logtime+delay_sec', u.rad.elevonL');
  u.smooth.elevonR = foh(u.logtime+delay_sec', u.rad.elevonR');
  u.smooth.throttle= foh(u.logtime+delay_sec', u.rad.throttle');
  
  t0 = max(min(est.logtime), min(min(u.logtime+delay_sec), min(airspeed.logtime)));
  tf = min(max(est.logtime), max(max(u.logtime+delay_sec), max(airspeed.logtime)));

  t = t0:dt:tf;
  
  rpy_airspeed0  = [ ppval(t, roll_s); ppval(t, pitch_s); ppval(t, yaw_s); ppval(t, airspeed_s)]';
  u0 = [ ppval(t, u.smooth.elevonL); ppval(t, u.smooth.elevonR); ppval(t, u.smooth.throttle) ]';

  data = iddata(rpy_airspeed0, u0, dt);
  
  set(data, 'InputName',{'elevL', 'elevR', 'throttle'}, 'OutputName',{'roll','pitch','yaw', 'airspeed'})

end