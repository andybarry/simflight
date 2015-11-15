function tvlqr = TrimTvlqr(start_time, end_time, tvlqr_in)

  if start_time < tvlqr_in.logtime(1)
    start_time = tvlqr_in.logtime(1);
  end

  if end_time > tvlqr_in.logtime(end)
    end_time = tvlqr_in.logtime(end);
  end

  tvlqr.utime = TrimTimes(start_time, end_time, tvlqr_in.logtime, tvlqr_in.utime);
  tvlqr.trajectory_number = TrimTimes(start_time, end_time, tvlqr_in.logtime, tvlqr_in.trajectory_number);
  
  tvlqr.logtime = TrimTimes(start_time, end_time, tvlqr_in.logtime, tvlqr_in.logtime);

end