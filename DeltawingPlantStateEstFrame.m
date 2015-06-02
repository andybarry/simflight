classdef DeltawingPlantStateEstFrame < DrakeSystem
% Defines the dynamics for the TBS Capi delta wing in the state estimator
% frame.  Use this for TVLQR but NOT FOR ANYTHING ELSE!

  properties
    p = {}; % DeltawingPlant
  end
  
  methods
    function obj = DeltawingPlantStateEstFrame(p)
      % @param parmaeters cell array of extra arguments to pass to
      % tbsc_model
      
      obj = obj@DrakeSystem(12,0,3,12,false,true);
      obj = setDirectFeedthrough(obj,0);
      obj = setOutputFrame(obj,getStateFrame(obj));
      
      obj = obj.setInputLimits(p.umin, p.umax); % input limits in [radians radians newtons]
      
      obj.p = p;
      
    end
    
    function [xdot, dxdot] = dynamics(obj, t, x, u)
      options = struct();
      options.grad_method = 'numerical';
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, dxdot] = geval(tempfunc, t, x, u, options);
    end
    
    function xdot = dynamics_no_grad(obj,t,x,u)
      
      x_drake_frame = ConvertStateEstimatorToDrakeFrame(x);
      
      xdot_drake_frame = obj.p.dynamics(t, x_drake_frame, u);
      
      xdot = ConvertXdotDrakeToStateEstimatorFrame(x_drake_frame, xdot_drake_frame);
      
      
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
      error('be super careful, you are attempting playback with a state-estimator-frame plant.  Probably you have plants mixed up.');
      
    end
    
    function playback_xtraj(xtraj, options)
      error('be super careful, you are attempting playback with a state-estimator-frame plant.  Probably you have plants mixed up.');
      
    end
      
    
    function v = constructVisualizer()
        error('be super careful, you are attempting playback with a state-estimator-frame plant.  Probably you have plants mixed up.');
    end
    
  end  
  
end
