function TrajectoryToDataComparisonPlotter(u, est, tvlqr_out, lib, xtrajsim, t_start, t_end, stable_traj_number)


  % determine how many trajectories are in this run
  [~, start_idx] = min(abs(tvlqr_out.logtime - t_start));
  
  % find the last index that is included in the times we have
  [~, end_idx] = min(abs(tvlqr_out.logtime - t_end));
  end_idx_t = tvlqr_out.logtime(end_idx);
  while (end_idx_t > t_end)
    end_idx = end_idx - 1;
    end_inx_t = tvlqr_out.logtime(end_idx);
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
  
  xlim_t_extra = 1.5;
  

  %%
  u = TrimU(t_start, t_end, u);
  est = TrimEst(t_start, t_end, est);


  %%
  dt = 1/140;
  
  stable_traj = lib.GetTrajectoryByNumber(stable_traj_number);
  trajectory_timeout_times = [];
  
  for i = 1 : length(traj)
    
    
    
    if (traj{i}.xtraj.tspan(2) ~= Inf)
%       if i < length(traj)
%         traj_end_t(i) = traj_start_t{i+1};
%       else
%         traj_end_t(i) = traj{i}.xtraj.tspan(2);
%       end
%       
%     else
%       if i < length(traj)
%         traj_end_t(i) = traj_start_t{i+1};
%       else
%         traj_end_t(i) = u.logtime(end) - u.logtime(1);
%       end
    end

    % this is the length of the TVLQR trajectory
    traj_delta_claimed = traj{i}.xtraj.tspan(2) - traj{i}.xtraj.tspan(1);
    
    % this is how long it ran for
    traj_delta_actual = traj_end_t(i) - traj_start_t(i);
    
    traj_t{i} = 0:dt:traj_delta_actual;
    
    
    if ~isnan(traj_delta_claimed) && ~isinf(traj_delta_claimed)
      % will be NaN for TILQR trajectories
      
      trajectory_timeout_times = [trajectory_timeout_times traj_delta_claimed + traj_start_t(i)];
      
      traj_eval_t = 0:dt:traj_delta_claimed;
      
      num_left = length(traj_t{i}) - length(traj_eval_t);
      
      if num_left > 0

        traj_x{i} = [ traj{i}.xtraj.eval(traj_eval_t) repmat(stable_traj.xtraj.eval(0), 1, num_left) ];
        traj_u{i} = [ traj{i}.utraj.eval(traj_eval_t) repmat(stable_traj.utraj.eval(0), 1, num_left) ];
        
      else
        % case where the trajectory was aborted before it finished
        traj_x{i} = traj{i}.xtraj.eval(traj_t{i});
        traj_u{i} = traj{i}.utraj.eval(traj_t{i});
      end
      
      
    else
      % TILQR case
      traj_x{i} = traj{i}.xtraj.eval(traj_t{i});
      traj_u{i} = traj{i}.utraj.eval(traj_t{i});
    end
    

    

    if ~isempty(xtrajsim)
      trajsim{i} = xtrajsim.eval(traj_t{i}+est.logtime(1));
    else
      trajsim{i} = zeros(12,1);
    end
  end
  
  % pack up data
  traj_data.traj_t = traj_t;
  traj_data.traj_x = traj_x;
  traj_data.traj_u = traj_u;
  traj_data.traj = traj;
  traj_data.traj_start_t = traj_start_t;
  traj_data.traj_end_t = traj_end_t;
  traj_data.trajectory_timeout_times = trajectory_timeout_times;
  traj_data.title_str = title_str;

  %% plot roll
  figure_num = 1;
  coordinate_num = 4;
  label = 'Roll (deg)';
  vals = rad2deg(est.orientation.roll);

  PlotComparison(est, traj_data, figure_num, coordinate_num, label, vals, 0, rad2deg(1), false);
 
  %% plot pitch

  figure_num = 2;
  coordinate_num = 5;
  label = 'Pitch (deg)';
  vals = rad2deg(est.orientation.pitch);
  
  PlotComparison(est, traj_data, figure_num, coordinate_num, label, vals, 0, rad2deg(1), true);

  %% plot z
  
  figure_num = 3;
  coordinate_num = 3;
  label = 'z (ft)';
  vals = est.pos.z * 3.28084;
  traj_add = est.pos.z(1) * 3.28084;
  
  PlotComparison(est, traj_data, figure_num, coordinate_num, label, vals, traj_add, 3.28084, false);
  
  
  PlotComparison(est, traj_data, 4, 1, 'x (meters)', est.pos.x, est.pos.x(1), 0, false);
  PlotComparison(est, traj_data, 5, 2, 'y (meters)', est.pos.y, est.pos.y(1), 0, false);
  return;
  

  %% plot u

  figure(4)
  clf
  plot(u.logtime, u.rad.elevonL);
  hold on
  plot(t+u.logtime(1), traju(1,:),'b--')



  plot(u.logtime, u.rad.elevonR, 'm-');
  plot(t+u.logtime(1), traju(2,:),'m--')
  %plot(u.logtime, u.throttle, 'k-');
  legend('elevonL', 'elevonL-plan', 'elevonR', 'elevonR-plan');
  grid on

  xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)  + xlim_t_extra]);
  title([ title_str ', u']);


end