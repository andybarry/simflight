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
    
    function WriteToFile(obj, filename, dt, overwrite_files)
      %
      % Write files that contain the trajectory information in filename and
      % filename-u.csv
      %
      % @param filename to write
      % @param dt sampling time for trajectory
      % @param overwrite_files set to true to overwrite
      %   @default false
      
      if nargin < 4
        overwrite_files = false;
      end

      u_filename = [filename(1:end-4) '-u.csv'];
      
      if ~overwrite_files && exist(filename, 'file') ~= 0
        error(['Not writing trajectory since "' filename '" exists.']);
      end
      
      if ~overwrite_files && exist(u_filename, 'file') ~= 0
        error(['Not writing trajectory since "' u_filename '" exists.']);
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

      % write all the xpoints to a file
      
      

      disp(['Writing: ' filename]);
      csvwrite(filename, xpoints');
      disp(['Writing: ' u_filename]);
      csvwrite(u_filename, upoints');

    end
    
  end
  
end