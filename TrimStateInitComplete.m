function state_init_complete = TrimStateInitComplete(start_time, end_time, state_init_complete_in)

  state_init_complete.utime = TrimTimes(start_time, end_time, state_init_complete_in.logtime, state_init_complete_in.utime);
  state_init_complete.logtime = TrimTimes(start_time, end_time, state_init_complete_in.logtime, state_init_complete_in.logtime);

end