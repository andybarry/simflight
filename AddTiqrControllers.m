function lib = AddTiqrControllers(lib, name, A, B, x0, u0, gains)
 
  Q = gains.Q;
  R = gains.R;
  K_pd = gains.K_pd;
  K_pd_yaw = gains.K_pd_yaw;
  K_pd_aggressive_yaw = gains.K_pd_aggressive_yaw;
  
  number = 10000;
  p = lib.p;
  
  % first just use the K_pd's and build trajectories
  
  %{
  ktraj = ConstantTrajectory(-K_pd);
  affine_traj = ConstantTrajectory(zeros(3,1));

  lqrsys = struct();
  lqrsys.D = ktraj;
  lqrsys.y0 = affine_traj;
  
  trajname = ['pd-no-yaw'];
  
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3)]);
    
    
  traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), trajname, comments);
  
  traj.WriteToFile(['trajlib/' trajname '-' num2str(number)], .01, true);
  
  number = number + 1;
  %}
  lib = lib.AddStabilizationTrajectory(x0, u0, K_pd, 'pd-no-yaw');
  
  %{
  
  
  ktraj = ConstantTrajectory(-K_pd_yaw);
  lqrsys = struct();
  lqrsys.D = ktraj;
  lqrsys.y0 = affine_traj;
  trajname = ['pd-yaw'];
  
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3)]);
    
  traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), trajname, comments);
  
  traj.WriteToFile(['trajlib/' trajname '-' num2str(number)], .01, true);
  
  number = number + 1;
  %}
  lib = lib.AddStabilizationTrajectory(x0, u0, K_pd_yaw, 'pd-yaw');
  
  
  %{
  ktraj = ConstantTrajectory(-K_pd_aggressive_yaw);
  lqrsys = struct();
  lqrsys.D = ktraj;
  lqrsys.y0 = affine_traj;
  
  trajname = ['pd-aggressive-yaw'];
  
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3)]);
    
  traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), trajname, comments);
  
  traj.WriteToFile(['trajlib/' trajname '-' num2str(number)], .01, true);
  
  number = number + 1;
  %}
  lib = lib.AddStabilizationTrajectory(x0, u0, K_pd_aggressive_yaw, 'pd-aggressive-yaw');
  
  
  

  K = lqr(full(A), full(B), Q, R);
  
  
  % kill everything that isn't pitch or roll
  %K(:, 1:3) = 0; % kill xyz
  %K(:, 6) = 0; % yaw
  %K(:, 7) = 0; % airspeed
  %K(:, 8:9) = 0; % ydot zdot
  %K(:, 12) = 0; % yawdot
  
  K0 = K;
  K0(:, 1:3) = 0; % kill xyz
  K0(:, 6) = 0; % yaw
  K0(:, 7) = 0; % airspeed
  K0(:, 8:9) = 0; % ydot zdot
  K0(:, 12) = 0; % yawdot
  
  %{
  ktraj = ConstantTrajectory(-K0);
  lqrsys = struct();
  lqrsys.D = ktraj;
  lqrsys.y0 = affine_traj;
  
  trajname = [name '-just-roll-pitch'];
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3) ...
      prettymat('Q', Q, 5) prettymat('R', R)]);
  traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), trajname, comments);
  traj.WriteToFile(['trajlib/' trajname '-' num2str(number)], .01, true);
  number = number + 1;
  %}
  trajname = [name '-just-roll-pitch'];
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3) ...
      prettymat('Q', Q, 5) prettymat('R', R)]);
    
  lib = lib.AddStabilizationTrajectory(x0, u0, K0, trajname, comments);
  
  
  
  K1 = K;
  K1(:, 1:3) = 0; % kill xyz
  %K1(:, 6) = 0; % yaw
  K1(:, 7) = 0; % airspeed
  K1(:, 8:9) = 0; % ydot zdot
  K1(:, 12) = 0; % yawdot
  
  %{
  ktraj = ConstantTrajectory(-K1);
  lqrsys = struct();
  lqrsys.D = ktraj;
  lqrsys.y0 = affine_traj;
  
  trajname = [name '-just-roll-pitch-yaw'];
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(parameters), 3) ...
      prettymat('Q', Q, 5) prettymat('R', R)]);
  traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), trajname, comments);
  traj.WriteToFile(['trajlib/' trajname '-' num2str(number)], .01, true);
  number = number + 1;
  %}
  
  trajname = [name '-just-roll-pitch-yaw'];
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3) ...
      prettymat('Q', Q, 5) prettymat('R', R)]);
    
  lib = lib.AddStabilizationTrajectory(x0, u0, K1, trajname, comments);
  
  
  
  
  
  K2 = K;
  
  %{
  ktraj = ConstantTrajectory(-K2);
  lqrsys = struct();
  lqrsys.D = ktraj;
  lqrsys.y0 = affine_traj;
  
  trajname = [name '-full'];
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3) ...
      prettymat('Q', Q, 5) prettymat('R', R)]);
  traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), trajname, comments);
  traj.WriteToFile(['trajlib/' trajname '-' num2str(number)], .01, true);
  number = number + 1;
  %}
  
  trajname = [name '-full'];
  comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(p.parameters), 3) ...
      prettymat('Q', Q, 5) prettymat('R', R)]);
    
  lib = lib.AddStabilizationTrajectory(x0, u0, K2, trajname, comments);
  

end