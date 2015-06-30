classdef TrajectoryInLibrary
  % A class for trajectories inside the TrajectoryLibrary.
  
  properties
    xtraj;
    utraj;
    lqrsys; % LQR controller
    state_frame; % state frame of the plant
    body_coordinate_frame;
    name; % name to prepend in the filename
    comments; % comments to be written to a file
  end
  
  methods
    
    function obj = TrajectoryInLibrary(xtraj, utraj, lqrsys, state_frame, name, comments)
      % @param name (optional) name for the filename
      %   @default traj
      % @param comments (optional) string to put in the comments file.  Usually
      % something like prettymat('Q', Q)
      %   @default ''
      
      typecheck(xtraj, 'Trajectory');
      
      if isa(utraj, 'ConstantTrajectory')
        utraj = PPTrajectory(utraj);
      end
      
      typecheck(utraj, 'Trajectory');
      
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      
      obj.lqrsys = lqrsys;
      obj.state_frame = state_frame;
      
      if nargin < 4
        name = 'traj';
      end
      
      str = regexprep(name,'[^a-zA-Z0-9_-]','');
      
      if ~strcmp(str, name)
        error('Special characters not allowed in string names.');
      end
      
      obj.name = name;
      
      if nargin < 5
        comments = '';
      end
      
      obj.comments = comments;
      
      obj.body_coordinate_frame = CoordinateFrame('body_frame_delta', 12, 'x');
      
      % add transform to/from the body frame
      to_est_frame = @(~, ~, x) ConvertDrakeFrameToEstimatorFrame(x);
      to_drake_frame = @(~, ~, x) ConvertStateEstimatorToDrakeFrame(x);
      
      trans_to_est = FunctionHandleCoordinateTransform(12, 0, obj.state_frame, obj.body_coordinate_frame, true, true, to_est_frame, to_est_frame, to_est_frame);
      trans_to_drake = FunctionHandleCoordinateTransform(12, 0, obj.body_coordinate_frame, obj.state_frame, true, true, to_drake_frame, to_drake_frame, to_drake_frame);
      obj.state_frame.addTransform(trans_to_est);
      obj.body_coordinate_frame.addTransform(trans_to_drake);
      
      %obj.xtraj = obj.xtraj.setOutputFrame(obj.state_frame);
      
    end
    
    function playback(obj, p)
      % Plays the trajectory in a visualizer
      %
      % @param p DeltawingPlant
      
      p.playback(obj.xtraj, obj.utraj, struct('slider', true));
      
    end
    
    function draw(obj, options)
      % Draw the trajectory using LCMGL
      %
      % @param options options structure to pass to DrawTrajectoryLcmGl
      
      if nargin < 2
        options = struct();
        options.color = [1, 0, 0];
        options.switch_buffers = true;
        options.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'trajectory_library');
      end
      
        
      DrawTrajectoryLcmGl(obj.xtraj, 'trajectory_library', options);
    end
    
    
    function WriteToFile(obj, filename_prefix, dt, overwrite_files)
      %
      % Write files that contain the trajectory information in filename.csv and
      % filename-u.csv or filename-affine.csv or filename-controller.csv
      %
      % @param filename_prefix filename_prefix to write files to (aka
      % filename_prefix-u.csv)
      % @param dt sampling time for trajectory
      % @param overwrite_files set to true to overwrite
      %   @default false
      
      if nargin < 4
        overwrite_files = false;
      end

      state_filename = [filename_prefix '-x.csv'];
      
      u_filename = [filename_prefix '-u.csv'];
      
      controller_filename = [filename_prefix '-controller.csv'];
      
      affine_filename = [filename_prefix '-affine.csv'];
      
      comment_filename = [filename_prefix '-comments.txt'];
      
      if ~overwrite_files && exist(state_filename, 'file') ~= 0
        error(['Not writing trajectory since "' state_filename '" exists.']);
      end
      
      if ~overwrite_files && exist(u_filename, 'file') ~= 0
        error(['Not writing trajectory since "' u_filename '" exists.']);
      end
      
      if ~overwrite_files && exist(controller_filename, 'file') ~= 0
        error(['Not writing trajectory since "' controller_filename '" exists.']);
      end
      
      if ~overwrite_files && exist(affine_filename, 'file') ~= 0
        error(['Not writing trajectory since "' affine_filename '" exists.']);
      end
      
      if ~overwrite_files && exist(comment_filename, 'file') ~= 0
        error(['Not writing trajectory since "' comment_filename '" exists.']);
      end
      
      xpoints = [];

      breaks = obj.xtraj.getBreaks();
      endT = breaks(end);

      counter = 1;
      for t = 0:dt:endT
          xpoints(:,counter) = [t; obj.xtraj.eval(t)];
          counter = counter + 1;
      end

      upoints = [];

      counter = 1;
      for t = 0:dt:endT
          upoints(:,counter) = [t; obj.utraj.eval(t)];
          counter = counter + 1;
      end
      
      Kpoints = [];
      
      kpoint_headers = {'t'};
      
      for i = 1 : obj.utraj.dim
        for j = 1:12
          kpoint_headers{end+1} = ['k' num2str(i) '_' num2str(j)];
        end
      end
           
      
      counter = 1;
      for t = 0:dt:endT
        
        this_k = obj.lqrsys.D.eval(t);
        
        col_k = [];
        
        for i = 1 : obj.utraj.dim
          col_k = [ col_k, this_k(i,:) ];
        end
        
        Kpoints(:, counter) = [t col_k];
          
        counter = counter + 1;
        
      end
      
      affine_points = [];

      counter = 1;
      for t = 0:dt:endT
          affine_points(:,counter) = [t; obj.lqrsys.y0.eval(t)];
          counter = counter + 1;
      end
      

      % write all the xpoints to a file
      
      xpoint_headers = { 't', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'xdot', 'ydot', 'zdot', 'rolldot', 'pitchdot', 'yawdot'};

      disp(['Writing: ' state_filename]);
      TrajectoryInLibrary.csvwrite_wtih_headers(state_filename, xpoint_headers, xpoints');
      
      upoint_headers = { 't', 'elevL', 'elevR', 'throttle'};
      
      disp(['Writing: ' u_filename]);
      TrajectoryInLibrary.csvwrite_wtih_headers(u_filename, upoint_headers, upoints');
      
      
      disp(['Writing: ' controller_filename]);
      TrajectoryInLibrary.csvwrite_wtih_headers(controller_filename, kpoint_headers, Kpoints');
      
      affine_headers = { 't', 'affine_elevL', 'affine_elevR', 'affine_throttle' };
      
      disp(['Writing: ' affine_filename]);
      TrajectoryInLibrary.csvwrite_wtih_headers(affine_filename, affine_headers, affine_points');
      
      disp(['Writing: ' comment_filename]);
      fid = fopen(comment_filename, 'w');
      fprintf(fid, '%s', obj.comments);
      fclose(fid);
      

    end
    
    function converted_traj = ConvertToStateEstimatorFrame(obj)
      % Converts the object's internal trajectories into the frame used by
      % the onboard state estimator.
      %
      % @retval converted object

%       xtraj_convert = obj.xtraj.inFrame(obj.body_coordinate_frame);
%       lqrsys_convert = obj.lqrsys.inInputFrame(obj.body_coordinate_frame);
%       
%       converted_traj = TrajectoryInLibrary(xtraj_convert, obj.utraj, lqrsys_convert, obj.state_frame);
%       

      old_K = obj.lqrsys.D.eval(0);
      x_drake = obj.xtraj.eval(0);
      
      x_body = ConvertDrakeFrameToEstimatorFrame(x_drake);
      
      new_K = old_K * x_drake * x_body';


    end
    
    function converted_traj = ConvertToDrakeFrame(obj)
      % Converts the object's internal trajectories into the frame used by
      % Drake.
      %
      % @retval converted object
      
      xtraj_convert = obj.xtraj.inFrame(obj.state_frame);
      lqrsys_convert = obj.lqrsys.inInputFrame(obj.state_frame);
      
      converted_traj = TrajectoryInLibrary(xtraj_convert, obj.utraj, lqrsys_convert, obj.state_frame);
      
    end
    
  end
  
  methods (Static)
    
    function csvwrite_wtih_headers(filename, headers, array)
       
      assert(~isempty(headers), 'No headers?');
      
      head_str = headers{1};
      
      for i = 2 : length(headers)
        head_str = [ head_str, ', ', headers{i}];
      end
      
      fid = fopen(filename, 'w');
      fprintf(fid, '%s\r\n', head_str);
      fclose(fid);
      
      dlmwrite(filename, array, '-append', 'delimiter', ',');
      
    end
 
    
    
    
  end
  
end