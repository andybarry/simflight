function [t_start, t_end] = FindAutonomousFlight(log)
  % find large acceleration
  accel_start = FindActiveTimes(log.est.logtime, log.est.accel.x, 40);
  
  accel_start = accel_start(1);
  
  t_start = accel_start;
  
  norm_accel = sqrt(log.est.accel.x.*log.est.accel.x + log.est.accel.y.*log.est.accel.y + log.est.accel.z.*log.est.accel.z);
  
  accel_landing = FindActiveTimes(log.est.logtime, norm_accel, 50);

  % find switch to manual mode
  [~, end_autonomous] = FindActiveTimes(log.u.logtime, log.u.is_autonomous, 0.5);
  
  end_autonomous = end_autonomous(end);
  
  t_end = end_autonomous;
  
  clf
  plot(log.u.logtime, log.u.is_autonomous);
  hold on
  plot(log.est.logtime, log.est.pos.z);
  
  plot([t_start t_end], [10 10], 'k--');
  
%   for i = 1 : length(accel_landing)
%     if abs(accel_landing(i) - end_autonomous) < 1
%       t_end = accel_landing(i);
%       return;
%     end
%   end
%   error('i give up.');

end