function PlotComparison(est, traj_data, figure_num, coordinate_num, label, vals, traj_add, traj_multiply, flip_y_axis)
  
  figure(figure_num)
  clf

  plot(est.logtime, vals, '-b')
  hold all

  % loop through all of the trajectories in this grpah
  for i = 1 : length(traj_data.traj)
    
    this_traj_t = traj_data.traj_t{i};
    
    traj_coord = traj_data.traj_x{i}(coordinate_num,:);
    plot(this_traj_t+traj_data.traj_start_t(i), traj_add(i) + traj_multiply * traj_coord,'r-')

    %plot(t+traj_start{i}, rad2deg(trajsim(4,:)), '--k');
    
  end
  
  
  DrawLinesAtTimes(traj_data.traj_end_t, 'k--');
  DrawLinesAtTimes(traj_data.trajectory_timeout_times, 'k-.');

  first_traj_t = traj_data.traj_t{1};
  last_traj_t = traj_data.traj_t{length(traj_data.traj)};
  %xlim([first_traj_t(1)+est.logtime(1) last_traj_t(end)+est.logtime(1) + xlim_t_extra]);
  
  if flip_y_axis
    set(gca, 'YDir', 'reverse');
  end

  grid on
  xlabel('Time (s)');
  ylabel(label);
  legend('Actual', 'Planned')
  title([ label ': ' traj_data.title_str]);

end