classdef DeltawingPlant < DrakeSystem
% Defines the dynamics for the TBS Capi delta wing.

  properties
    parameters = {}; % extra arguments to pass to tbsc_model
  end
  
  methods
    function obj = DeltawingPlant(parameters)
      % @param parmaeters cell array of extra arguments to pass to
      % tbsc_model
      %   @default {}
      
      obj = obj@DrakeSystem(12,0,3,12,false,true);
      obj = setDirectFeedthrough(obj,0);
      obj = setOutputFrame(obj,getStateFrame(obj));
      
      obj = obj.setInputLimits([-0.90; -0.90; 0], [0.855; 0.855; 5.33976]); % input limits in [radians radians newtons]
      
      if nargin > 0
        obj.parameters = parameters;
      end
      
    end
    
    function [xdot, dxdot] = dynamics(obj, t, x, u)
      options = struct();
      options.grad_method = 'numerical';
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, dxdot] = geval(tempfunc, t, x, u, options);
    end
    
    function xdot = dynamics_no_grad(obj,t,x,u)
      
      x = ConvertToModelFrameFromDrakeWorldFrame(x);
      
      xdot_model_frame = tbsc_model(t, x, u, obj.parameters{:});
      
      xdot = ConvertXdotModelToDrake(x, xdot_model_frame);
      
      
    end
    
    function [y,dy] = output(obj,t,x,u)
      y = x;
      if (nargout>1)
        dy=[zeros(obj.num_y,1),eye(obj.num_y),zeros(obj.num_y,obj.num_u)];
      end
    end
    
    function x = getInitialState(obj)
      x = zeros(12,1);
    end
    
  end
  
  methods (Static)
    
    function playback(xtraj, utraj, options)
      if nargin < 3
        options = struct();
      end
      
      v = DeltawingPlant.constructVisualizer;
      
      traj_and_u = [xtraj; utraj];

      fr = traj_and_u.getOutputFrame();

      transform_func = @(t, x, x_and_u) [ x_and_u(1:6); x_and_u(15); x_and_u(13:14); x_and_u(7:12); zeros(3,1)];

      trans = FunctionHandleCoordinateTransform(17, 0, traj_and_u.getOutputFrame(), v.getInputFrame(), true, true, transform_func, transform_func, transform_func);

      fr.addTransform(trans);

      warning('off', 'Drake:FunctionHandleTrajectory');
      playback(v, traj_and_u, options);
      warning('on', 'Drake:FunctionHandleTrajectory');
      
    end
    
    function playback_xtraj(xtraj, options)
      if nargin < 2
        options = struct();
      end
      
      utraj = ConstantTrajectory([0; 0; 0]);
      
      DeltawingPlant.playback(xtraj, utraj, options);
      
    end
      
    
    function v = constructVisualizer()
        options.floating = true;
        r = RigidBodyManipulator('TBSC_visualizer.urdf', options);
        v = HudBotVisualizer(r);
    end
    
  end  
  
end
