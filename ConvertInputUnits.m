function u = ConvertInputUnits(u_in)
  error('this function is outdated.  Look at loadDeltawing and ReadSimpleConfigServos.m');
  % convert from microsecond command pulse lengths to radians and
  % percentage
  
  u = u_in;
  
  u.elevonL_command = u_in.elevonL;
  u.elevonR_command = u_in.elevonR;
  u.throttle_command = u_in.throttle;
  
  u.elevonL = 0.002114 * u_in.elevonL - 3.011;
  
  u.elevonR = -0.0023 * u_in.elevonR + 3.688;
  
  u.throttle = max(0.01004 * u_in.throttle - 12.17, 0);


end