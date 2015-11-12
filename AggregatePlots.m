for i = 1 : length(logs)
  disp(i)
  [t_start, t_end] = FindAutonomousFlight(logs(i))
  pause
end
