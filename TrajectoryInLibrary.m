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
    
    function WriteToFile(obj, filename, dt)

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
      
      u_filename = [filename(1:end-4) '-u.csv'];

      disp(['Writing: ' filename]);
      csvwrite(filename, xpoints');
      disp(['Writing: ' u_filename]);
      csvwrite(u_filename, upoints');

    end
    
  end
  
end