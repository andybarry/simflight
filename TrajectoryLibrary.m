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
    
    function obj = AddTrajectory(obj, xtraj, utraj, lqrsys)
      % Adds a trajectory to the library
      %
      % @param xtraj state trajectory
      % @param utraj input trajectory
      % @param lqrsys LTV system that implements TVLQR controller
      %
      % @retval obj updated object
      
      
      
      obj.trajectories{end+1} = TrajectoryInLibrary(xtraj, utraj, lqrsys);
      
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
    
    
  end
  
  
  
end