function TrajectoryToDataComparisonPlotter(u, est, tvlqr_out, lib, t_start, t_end, stable_traj_number)


  % determine how many trajectories are in this run
  [~, start_idx] = min(abs(tvlqr_out.logtime - t_start));
  
  % find the last index that is included in the times we have
  [~, end_idx] = min(abs(tvlqr_out.logtime - t_end));
  end_idx_t = tvlqr_out.logtime(end_idx);
  while (end_idx_t > t_end)
    end_idx = end_idx - 1;
    end_idx_t = tvlqr_out.logtime(end_idx);
  end
  
  traj_start_t = tvlqr_out.logtime(start_idx:end_idx);
  traj_end_t = [ traj_start_t(2:end); t_end ];
  

  % list trajectories
  traj_nums = tvlqr_out.trajectory_number(start_idx:end_idx);
  
  title_str = 'Trajectory #:';
  
  for i = 1:length(traj_nums)
    
    this_traj_num = traj_nums(i);
    
    traj{i} = lib.GetTrajectoryByNumber(this_traj_num);
    
    if i > 1
      title_str = [title_str ' --> '];
    end
    
    title_str = [title_str traj{i}.name  ' (' num2str(this_traj_num) ')'];
    
  end

 
  
  disp(['t: ' num2str(t_start) ' -- ' num2str(t_end)]);
  disp(title_str);


  %%
  dt = 1/140;
  
  stable_traj = lib.GetTrajectoryByNumber(stable_traj_number);
  
  % check for additional trajectories because of timeout
  for i = 1 : length(traj)

    % this is the length of the TVLQR trajectory
    length_of_trajectory = traj{i}.xtraj.tspan(2) - traj{i}.xtraj.tspan(1);
    
    % this is how long it ran for
    time_trajectory_ran = traj_end_t(i) - traj_start_t(i);
    
    
    if ~isnan(length_of_trajectory) && ~isinf(length_of_trajectory)
      % will be NaN for TILQR trajectories
      
      % TVLQR case
      
      % check to see if the trajectory:
      %   1) timed out and revereted to stabilization
      %         or
      %   2) was cut off eary
      
      
      if length_of_trajectory < time_trajectory_ran
        % timed out and reverted to stabilization
        
        % in this case, there's actually another trajectory in here
        % for when it jumped to TILQR
        
        traj = { traj{1:i}, stable_traj, traj{i+1:end} };
        traj_start_t = [traj_start_t(1:i); traj_start_t(i) + length_of_trajectory; traj_start_t(i+1:end)];
        traj_end_t = [ traj_start_t(2:end); t_end ];
          
        
      else
        
        % was aborted early
        
        % no new trajectories need be added
        
        
      end
      
    end
    
  end
  
  
  for i = 1 : length(traj)
    
    % evaluate each trajectory
    traj_t{i} = 0:dt:traj_end_t(i) - traj_start_t(i);
    
    traj_x{i} = traj{i}.xtraj.eval(traj_t{i});
    traj_u{i} = traj{i}.utraj.eval(traj_t{i});
    
    % for some reason the utraj evaluates weirdly for the TILQR case
    % and creates a three dimensional array -- fix that
    if (length(size(traj_u{i})) > 2)
      traj_u{i} = reshape(traj_u{i}, size(traj_u{i}, 1), size(traj_u{i}, 3));
    end

  end
 

  
  %% plot each part
  num_plots = 5;
  fig_size_y = 750;
  
  for i = 1 : num_plots
    
    figure(i);
    clf;
    
    fig_size = get(gcf, 'Position');
    delta_y = fig_size_y - fig_size(4);
    
    set(gcf, 'Position', [fig_size(1) fig_size(2)-delta_y fig_size(3) fig_size_y]);
  end
  
  for i = 1 : length(traj)
    
    this_start = traj_start_t(i);
    this_end = traj_end_t(i);
    
    u_trim = TrimU(this_start, this_end, u);
    est_trim = TrimEst(this_start, this_end, est);
    
    PlotTrajectoryPart(est_trim, u_trim, traj_t{i} + traj_start_t(i), traj_x{i}, traj_u{i}, title_str, true);
    
  end
  
  for i = 1 : num_plots
      figure(i);
      
      DrawLinesAtTimes(traj_end_t, 'k--');
      drawnow
  end


end