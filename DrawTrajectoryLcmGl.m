function DrawTrajectoryLcmGl(traj, lcmgl_name)
  % Draws a trajectory in LCMGL
  %
  % @param traj trajectory to draw
  % @param lcmgl_name string describing the trajectory
  %   @default 'trajectory'
  
  checkDependency('lcmgl');
  
  if nargin < 2
    lcmgl_name = 'trajectory';
  end
    
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), lcmgl_name);
  
  lcmgl.glColor3f(0,1,0);
  lcmgl.glLineWidth(2);

  last_knot = [];

  for knot = traj.eval(traj.getBreaks());
    
    lcmgl.sphere(knot, 0.05, 20, 20);

    if ~isempty(last_knot)
      lcmgl.line3(last_knot(1), last_knot(2), last_knot(3), knot(1), knot(2), knot(3));
    end

    last_knot = knot;
  end

  lcmgl.switchBuffers();
end
