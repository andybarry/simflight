function [elevL_command, elevR_command, throttle_command] = RadiansToServoCommands(elevL, elevR, throttle)

  % convert from microsecond command pulse lengths to radians and
  % percentage
  
  elevL_command = -473.037*(-elevL-3.011);
  
  elevR_command = 434.783*(3.688-elevR);
  
  throttle_command = -99.6016 * (-throttle - 12.17);


end