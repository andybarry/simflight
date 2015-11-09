classdef TrajectoryLibrary
  % A trajectory library for use in online control systems.  Supports
  % adding trajectories to the library and querying trajectories for
  % collision free paths
  
  properties
    p; % the plant
    trajectories = {};
    ti_rollout_time = 3.0; % time in seconds for all TI rollouts that are used for collision checking online
  end
  
  
  methods
    
    function obj = TrajectoryLibrary(plant)
      % Creates a trajectory library
      %
      % @param plant plant that the library is built for (some kind of DrakeSystem)
      
      assert(isa(plant, 'DeltawingPlantStateEstFrame'), 'plant is not a DeltawingPlantStateEstFrame.');
      
      obj.p = plant;
    end
    
    function obj = AddTrajectory(obj, xtraj, utraj, lqrsys, name, comments)
      % Adds a trajectory to the library
      %
      % 
      % @param xtraj state trajectory
      % @param utraj input trajectory
      % @param lqrsys LTV system that implements TVLQR controller
      % @param name (optional) prepending name for filename
      % @param comments (optional) comments to be put in a txt file when
      % the trajectory is written.  Useful for writing down, Q, R, model
      % parameters, etc.
      %
      % @retval obj updated object
      
      if nargin < 5
        name =  'traj';
      end
      
      if nargin < 6
        comments =  '';
      end
      
      
      obj.trajectories{end+1} = TrajectoryInLibrary(xtraj, utraj, lqrsys, obj.p.getStateFrame(), name, comments);
      
    end
    
    function obj = InsertExistingTrajectory(obj, traj, traj_num)
      % Adds a trajectory to the library
      %
      % @param traj TrajectoryInLibrary object
      % @param traj_num (optional) what number it should be
      %
      % @retval obj updated object
      
      if nargin < 3
        obj.trajectories{end+1} = traj;
      else
        
        index = TrajectoryLibrary.IndexFromTrajectoryNumber(traj_num);
        obj.trajectories = [obj.trajectories(1:index-1) {traj} obj.trajectories(index:end)];
        
      end
    end
    
    function obj = RenameTrajectory(obj, traj_num, new_name)
      traj = obj.GetTrajectoryByNumber(traj_num);
      traj = traj.Rename(new_name);
      
      obj.trajectories{TrajectoryLibrary.IndexFromTrajectoryNumber(traj_num)} = traj;
      
    end
    
    function obj = RemoveTrajectroy(obj, traj_nums)
      traj_nums = sort(traj_nums, 'descend');
      for i = 1 : length(traj_nums)
          num = traj_nums(i);
          obj.trajectories(TrajectoryLibrary.IndexFromTrajectoryNumber(num)) = [];
      end
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
    
    function ListTrajectories(obj)
      % Lists all trajectories
      
      for i = 1 : length(obj.trajectories)
        val = i - 1;
        disp([num2str(val) ': ' obj.GetTrajectoryByNumber(val).name]);
      end
      
    end
    
    function playback(obj, traj_num_from_filename)
      % Plays the trajectory in a visualizer
      %
      % @param traj_num_from_filename the trajectory number (as given by
      %   filename)
      
      traj = obj.GetTrajectoryByNumber(traj_num_from_filename);
      
      traj.playback(obj.p);
    end
    
    function [ytraj, xtraj, utraj] = SimulateTrajectory(obj, traj_num_from_filename, tf, x0)
      
      traj = obj.GetTrajectoryByNumber(traj_num_from_filename);
      
      options = struct();
      options.color = [1, 0, 0];
      DrawTrajectoryLcmGl(traj.xtraj, ['Nominal: ' traj.name], options);
      
      if nargin < 3 || tf < 0
        if traj.IsTimeInvariant()
          tf = 1;
        else
          tf = traj.xtraj.tspan(2);
        end
      end
      
      x0_str = 'custom';
      
      if nargin < 4
        x0_est = traj.xtraj.eval(0);
        x0 = ConvertStateEstimatorToDrakeFrame(x0_est);
        x0_str = 'default';
      end
      
      lqrsys = traj.lqrsys;
      
      fb_sys = feedback(obj.p.p, lqrsys);
      
      disp(['Simulating: "' traj.name '" for ' num2str(tf) ' second(s) with ' x0_str ' initial conditions...']);
      [ytraj, xtraj] = fb_sys.simulate([0 tf], x0);
      disp('done.');
      
      % compute an estimate of utraj
      
      xtraj_in_est_frame = xtraj.inOutputFrame(lqrsys.getInputFrame());
      lqrsys_in_drake_frame = lqrsys.inOutputFrame(obj.p.getInputFrame());
      dt = 0.01;
      t = xtraj.tspan(1) : dt : xtraj.tspan(2);
      for i = 1 : length(t)

        u(:,i) = lqrsys_in_drake_frame.output(t(i), [], xtraj_in_est_frame.output(t(i), [], []));
        
      end
      
      u_spline = spline(t, u);
      
      utraj = PPTrajectory(u_spline);
      
      DrawTrajectoryLcmGl(xtraj, ['Sim: ' traj.name]);
      
      
      obj.p.playback(xtraj, utraj, struct('slider', true));
      
    end
    
    function [obj, new_traj_num] = AddStabilizationTrajectory(obj, x0, u0, K_to_be_negated, trajname, comments, xtraj_rollout)
      
      if (nargin < 6 || isempty(comments))
        comments = sprintf('%s\n\n%s', [trajname, prettymat('Parameters', cell2mat(obj.p.parameters), 3)]);
      end
      
      if (nargin < 7)
        xtraj_rollout = [];
      end
      
      xtraj = ConstantTrajectory(x0);
      utraj = ConstantTrajectory(u0);
      
      ktraj = ConstantTrajectory(-K_to_be_negated);
      affine_traj = ConstantTrajectory(zeros(3,1));

      lqrsys = AffineSystem([],[],[],[],[], [], [], ktraj, affine_traj);
      
      lqrsys = setInputFrame(lqrsys,CoordinateFrame([obj.p.getStateFrame.name,' - ', mat2str(x0,3)],length(x0), obj.p.getStateFrame.prefix));
      
      obj.p.getStateFrame.addTransform(AffineTransform(obj.p.getStateFrame,lqrsys.getInputFrame,eye(length(x0)),-x0));
      lqrsys.getInputFrame.addTransform(AffineTransform(lqrsys.getInputFrame,obj.p.getStateFrame,eye(length(x0)),+x0));

      
      lqrsys = setOutputFrame(lqrsys,CoordinateFrame([obj.p.getInputFrame.name,' + ',mat2str(u0,3)],length(u0),obj.p.getInputFrame.prefix));
      lqrsys.getOutputFrame.addTransform(AffineTransform(lqrsys.getOutputFrame,obj.p.getInputFrame,eye(length(u0)),u0));
      obj.p.getInputFrame.addTransform(AffineTransform(obj.p.getInputFrame,lqrsys.getOutputFrame,eye(length(u0)),-u0));
  
%       lqrsys = lqrsys.setInputFrame(obj.p.getOutputFrame());

      traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, obj.p.getStateFrame(), trajname, comments);
      
      obj.trajectories{end+1} = traj;
      
      traj_num = length(obj.trajectories) - 1;
      
      if isempty(xtraj_rollout)
        % simulate for a rollout
        [~, simtraj] = obj.SimulateTrajectory(traj_num, obj.ti_rollout_time);
      
        % simulation is in Drake frame, covert to StateEstimatorFrame
        simtraj = ConvertXtrajFromDrakeFrameToStateEstFrame(simtraj);
      else
        simtraj = xtraj_rollout;
      end
      
      traj.xtraj = simtraj;
      
      % replace the object with the updated one
      obj.trajectories{end} = traj;
      
      new_traj_num = traj_num;
      
    end
    
    function traj = GetTrajectoryByNumber(obj, traj_num_from_filename)
      if (traj_num_from_filename > 1000)
        traj_num_from_filename = 12;
      end
      index = TrajectoryLibrary.IndexFromTrajectoryNumber(traj_num_from_filename);
      traj = obj.trajectories{index};
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
        numstr = sprintf('%05d', i-1);
        obj.trajectories{i}.WriteToFile([filename_prefix '-' obj.trajectories{i}.name '-' numstr], dt, overwrite_files);
      end
      
      % write a .mat file containing this object
      mat_filename = [filename_prefix '.mat'];
      
      % check for existing mat file
      if ~overwrite_files && exist(mat_filename, 'file') ~= 0
        error(['Not writing trajectory .mat since "' mat_filename '" exists.']);
      end
      
      disp(['Writing: ' mat_filename]);
      lib = obj;
      save(mat_filename, 'lib');
      
      disp('done.');
      
    end
    
    function [best_traj, max_dist] = FindFarthestTrajectoryLinear(obj, points, threshold, t_start, position)

      if nargin < 3
        threshold = 50;
      end
      
      if nargin < 4
        t_start = 0;
      end
      
      if nargin < 5
        position = [0; 0; 0; 0];
      else
        if length(position) ~= 4
          error('Position must be in the form [x; y; z; yaw]');
        end
      end

      % run nearest neighbor on each trajectory we want to use

      max_dist = -1;
      best_traj = -1;

      for i = 0 : length(obj.trajectories) - 1

        fprintf('Checking trajectory %d...', i);

        traj = obj.GetTrajectoryByNumber(i);

        dist = traj.NearestNeighborLinear(points, t_start, position);
        
        fprintf(' %f\n', dist);
        

        if (max_dist < 0 || dist > max_dist)
          max_dist = dist;
          best_traj = i;

          if ( max_dist > threshold )
            disp('exiting because we found a good enough trajectory');
            break;
          end

        end
      end
    end
    
  end
  
  methods (Static)
    function index = IndexFromTrajectoryNumber(traj_num)
      index = traj_num + 1;
    end
    
    function lib = RebuildTvControllers(old_lib, gains)
      % Creates a new library by rebuilding controllers using the existing
      % open loop trajectories.
      %
      % @param old_lib old library to use the open-loop trajectories from
      %
      % @retval new library
      
      if nargin < 2
        [~, gains] = GetDefaultGains();
      end
      
      lib = TrajectoryLibrary(old_lib.p);
      lib.ti_rollout_time = old_lib.ti_rollout_time;
      
      for i = 0 : length(old_lib.trajectories) - 1
        traj = old_lib.GetTrajectoryByNumber(i);
        if traj.IsTimeInvariant()
%            x0 = traj.xtraj.eval(0);
%            u0 = traj.utraj.eval(0);
%         
%           [A, B, C, D, xdot0, y0] = lib.p.linearize(0, x0, u0);
%           
%           lib = AddTiqrControllers(lib, traj.name, A, B, x0, u0, gains);
          %warning('just adding TI controller, not doing anything');
          lib.trajectories{end + 1} = traj;
        else
          traj2 = traj.StripControllerFromName();
          disp(['Rebuilding controllers for ' traj2.name '...']);
          lib = AddLqrControllersToLib(traj2.name, lib, traj2.xtraj, traj2.utraj, gains, true, true);
        end
      end
      
    end
  end
  
  
end