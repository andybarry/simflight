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
    
    function obj = AddTrajectory(obj, trajectory)
      % Adds a trajectory to the library
      %
      % @param trajectory trajectory to add
      %
      % @retval obj updated object
      
      typecheck(trajectory, 'PPTrajectory');
      
      obj.trajectories{end} = trajectory;
      
    end
    
    
  end
  
  
  
end