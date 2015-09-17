function lib = AddTiqrControllers(lib, name, A, B, x0, u0, gains)
 
  Q = gains.Q;

  [lib_just_for_rollout, traj_num] = lib.AddStabilizationTrajectory(x0, u0, gains.K_pd, [name '-pd-no-yaw']);
  xtraj_rollout = lib_just_for_rollout.GetTrajectoryByNumber(traj_num).xtraj;
  
  for i = 1 : length(gains.R_values)
      R = diag([gains.R_values(i) gains.R_values(i) gains.R_values(i)]);
      
      K = lqr(full(A), full(B), Q, R);

      trajname = [name '-R-' num2str(gains.R_values(i))];
      comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(lib.p.parameters), 3) ...
          prettymat('Q', Q, 5) prettymat('R', R)]);

      lib = lib.AddStabilizationTrajectory(x0, u0, K, trajname, comments, xtraj_rollout);
  end
  

end