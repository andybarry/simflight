classdef TrajectoryInLibrary
  % A class for trajectories inside the TrajectoryLibrary.
  
  properties
    xtraj;
    utraj;
  end
  
  methods
    
    function obj = TrajectoryInLibrary(xtraj, utraj)
      
      typecheck(xtraj, 'PPTrajectory');
      
      if isa(utraj, 'ConstantTrajectory')
        utraj = PPTrajectory(utraj);
      end
      
      typecheck(utraj, 'PPTrajectory');
      
      obj.xtraj = xtraj;
      obj.utraj = utraj;
      
    end
    
  end
  
end