classdef DeltawingPlantStateEstFrameTester < DrakeSystem
% Defines the dynamics for the TBS Capi delta wing in the state estimator
% frame.  Use this for TVLQR but NOT FOR ANYTHING ELSE!

  properties
    p_state_est = {}; % DeltawingPlantStateEstFrame
  end
  
  methods
    function obj = DeltawingPlantStateEstFrameTester(p_state_est)
      % @param parmaeters cell array of extra arguments to pass to
      % tbsc_model
      
      obj = obj@DrakeSystem(12,0,3,12,false,true);
      obj = setDirectFeedthrough(obj,0);
      obj = setOutputFrame(obj,getStateFrame(obj));
      
      obj = obj.setInputLimits(p_state_est.umin, p_state_est.umax); % input limits in [radians radians newtons]
      
      obj.p_state_est = p_state_est;
      
    end
    
    function [xdot, dxdot] = dynamics(obj, t, x, u)
      options = struct();
      options.grad_method = 'numerical';
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, dxdot] = geval(tempfunc, t, x, u, options);
    end
    
    function xdot_drake_frame = dynamics_no_grad(obj,t,x,u)
      
      x_est_frame = ConvertDrakeFrameToEstimatorFrame(x);
      
      
      xdot_state_est_frame = obj.p_state_est.dynamics(t, x_est_frame, u);
      
      xdot_model_frame = ConvertXdotStateEstimatorToModelFrame(xdot_state_est_frame);
      
      x_model_frame = ConvertToModelFrameFromDrakeWorldFrame(x);
      xdot_drake_frame = ConvertXdotModelToDrake(x_model_frame, xdot_model_frame);
      
      
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
