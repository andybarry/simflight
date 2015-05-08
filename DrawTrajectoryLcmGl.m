function DrawTrajectoryLcmGl(traj, lcmgl_name, options)
  % Draws a trajectory in LCMGL
  %
  % @param traj trajectory to draw
  % @param lcmgl_name string describing the trajectory
  %   @default 'trajectory'
  % @param options options:
  %   <pre>
  %     Options:
  %       sphere_size = 0.05
  %       color = [0, 1, 0] (green)
  %       switch_buffers = true
  %       lcmgl = <creates new lcmgl object, ignores lcmgl_name>
  %   </pre>
  
  
  checkDependency('lcmgl');
  
  if nargin < 2
    lcmgl_name = 'trajectory';
  end
  
  if nargin < 3
    options = struct();
  end
  
  if ~isfield(options, 'sphere_size'), options.sphere_size = 0.05; end
  if ~isfield(options, 'color'), options.color = [0, 1, 0]; end
  if ~isfield(options, 'switch_buffers'), options.switch_buffers = true; end
  
  if ~isfield(options, 'lcmgl')
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), lcmgl_name);
  else
    lcmgl = options.lcmgl;
  end
  
  lcmgl.glColor3f(options.color(1), options.color(2), options.color(3));
  lcmgl.glLineWidth(2);

  last_knot = [];

  if isa(traj, 'Trajectory')
    eval_array = traj.eval(traj.getBreaks());
  else
    eval_array = traj;
  end
  
  for knot = eval_array
    
    lcmgl.sphere(knot, options.sphere_size, 20, 20);
    
    % draw a coordinate system
    unit_x = 1*[1; 0; 0];
    unit_y = 1*[0; 1; 0];
    unit_z = 1*[0; 0; 1];
    
    % rotate the coordinate system to correspond with the trajectory
    rot_mat = rpy2rotmat(knot(4:6));
    
    rot_x = rot_mat * unit_x;
    rot_y = rot_mat * unit_y;
    rot_z = rot_mat * unit_z;
    
    %lcmgl.line3(knot(1), knot(2), knot(3), rot_x(1)+knot(1), rot_x(2)+knot(2), rot_x(3)+knot(3));
    %lcmgl.line3(knot(1), knot(2), knot(3), rot_y(1)+knot(1), rot_y(2)+knot(2), rot_y(3)+knot(3));
    lcmgl.line3(knot(1), knot(2), knot(3), rot_z(1)+knot(1), rot_z(2)+knot(2), rot_z(3)+knot(3));

    if ~isempty(last_knot)
      lcmgl.line3(last_knot(1), last_knot(2), last_knot(3), knot(1), knot(2), knot(3));
    end

    last_knot = knot;
  end

  if options.switch_buffers
    lcmgl.switchBuffers();
  end
end
