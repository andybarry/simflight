classdef TrajectoryLibrary
  % A trajectory library for use in online control systems.  Supports
  % adding trajectories to the library and querying trajectories for
  % collision free paths
  
  properties
    trajectories = {};
  end
  
  
  methods
    
    function obj = TrajectoryLibrary()
      
    end
    
    function obj = AddTrajectory(obj, p, xtraj, utraj, lqrsys)
      % Adds a trajectory to the library
      %
      % @param p plant (some kind of DrakeSystem)
      % @param xtraj state trajectory
      % @param utraj input trajectory
      % @param lqrsys LTV system that implements TVLQR controller
      %
      % @retval obj updated object
      
      
      
      obj.trajectories{end+1} = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame());
      
    end
    
    function DrawTrajectories(obj)
      
      options = struct();
      options.sphere_size = 0.01;
      options.color = [1, 0, 0];
      options.switch_buffers = false;
      options.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'trajectory_library');
      
      for i = 1:length(obj.trajectories)
        
        if i == length(obj.trajectories)
          options.switch_buffers = true;
        end
        
        DrawTrajectoryLcmGl(obj.trajectories{i}.xtraj, 'trajectory_library', options);
      end
      
    end
    
    function WriteToFile(obj, filename_prefix, overwrite_files)
      % generate the data files for the C++, onboard code
      % files will be called: filename_prefix-00001.csv
      %
      % @param filename_prefix prefix for trajectories
      % @param overwrite_files set to true to overwrite
      %   @default false
      
      if nargin < 3
        overwrite_files = false;
      end
      
      dt = 0.01;
      
      disp('Writing data files...');
      
      for i = 1 : length(obj.trajectories)
        numstr = sprintf('%05d', i);
        obj.trajectories{i}.WriteToFile([filename_prefix '-' numstr], dt, overwrite_files);
      end
      
      disp('done.');
      
    end
    
    
  end
  
  
end