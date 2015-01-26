classdef TrajectoryInLibrary
  % A class for trajectories inside the TrajectoryLibrary.
  
  properties
    xtraj;
    utraj;
    lqrsys; % LQR controller
  end
  
  methods
    
    function obj = TrajectoryInLibrary(xtraj, utraj, lqrsys)
      
      typecheck(xtraj, 'PPTrajectory');
      
      if isa(utraj, 'ConstantTrajectory')
        utraj = PPTrajectory(utraj);
      end
      
      typecheck(utraj, 'PPTrajectory');
      
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      
      obj.lqrsys = lqrsys;
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
      

    end
    
    function ConvertToStateEstimatorFrame(obj)
      body_coordinate_frame = CoordinateFrame('body_frame_delta', 12, 'x');
      
      state_frame = obj.xtraj.getStateFrame();

      transform_func = @(~, ~, x) TrajectoryInLibrary.ConvertStateToStateEstimatorState(x);

      trans = FunctionHandleCoordinateTransform(12, 0, bj.xtraj.getStateFrame(), body_coordinate_frame, true, true, transform_func, transform_func, transform_func);

      
      state_frame.addTransform(trans);
      
      
      % todo
      
      
      
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
    
    function x = ConvertStateToStateEstimatorState(x_drake_frame)
      % Converts the 12-dimensional vector for the aircraft's state into
      % one that works for the state esimator and should be exported and
      % used for online control
      %
      % @param x_drake_frame input state
      %
      % @retval x output state
      
      % global position stays in the global frame
      
      % Drake frame:
      % x(1:3): x,y,z, in global frame (ENU coordinates)
      % x(4:6): rpy
      % x(7:9): xdot, ydot, zdot, in global frame (ENU coordinates)
      % x(10:12): rdot, pdot, ydot
      
      % State estimator frame:
      %
      % double pos[3];              // position x,y,z in meters in local frame (ENU coordinates)
      % double vel[3];              // velocity in m/s, expressed in body frame
      % 
      % double orientation[4];      // rotate vector in body coordinate frame 
      %                             // (X-forward, Z-down) by this to get that vector
      %                             // in local frame
      % 
      % double rotation_rate[3];    // angular velocity vector of the vehicle
      %                             // in rad/s.  This is expressed in the body
      %                             // frame.
      
      
      x(1:3) = x_drake_frame;
      
      
      %x(4:6) = 
      
      x(7:9) = ConvertGlobalVelocityToBodyVelocity( %todo
      
    end
    
  end
  
end