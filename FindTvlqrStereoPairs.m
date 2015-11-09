% Finds stereo and trajectory reports nearby each other

% search through log time

clear paired_times
threshold = 1;
counter = 1;
for i = 1 : length(stereo.logtime)
  
  this_t = stereo.logtime(i);
  
  [val, idx] = min(abs(tvlqr.logtime - this_t));
  
  if abs(val) < threshold
  
    tvlqr_t = tvlqr.logtime(idx);

    pair = struct();
    pair.tvlqr_t = tvlqr_t;
    pair.stereo_t = this_t;
    pair.delta_t = tvlqr_t - this_t;

    paired_times(counter) = pair;

    counter = counter + 1;
  end
end


val = 0;
for i = 1 : length(paired_times)
  val = val + paired_times(i).delta_t;
end
average_delay = val/length(paired_times);

disp(['Average delay over ' num2str(length(paired_times)) ' is ' num2str(average_delay) ' sec or ' num2str(average_delay*1000) ' ms']);