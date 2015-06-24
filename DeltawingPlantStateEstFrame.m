classdef DeltawingPlantStateEstFrame < DrakeSystem
% Defines the dynamics for the TBS Capi delta wing in the state estimator
% frame.  Use this for TVLQR but NOT FOR ANYTHING ELSE!

  properties
    p = {}; % DeltawingPlant
    parameters;
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
      obj.parameters = obj.p.parameters;
      
      % tell the DeltawingPlant how to convert from its frame to this frame
      drake_to_est_func = @(t, x, u) ConvertDrakeFrameToEstimatorFrame(u);
      est_to_drake_func = @(t, x, u) ConvertStateEstimatorToDrakeFrame(u);
      trans_drake_to_state_est = FunctionHandleCoordinateTransform(12, 0, obj.p.getStateFrame(), obj.getStateFrame(), false, true, drake_to_est_func, drake_to_est_func, drake_to_est_func);
      trans_state_est_to_drake = FunctionHandleCoordinateTransform(12, 0, obj.getStateFrame(), obj.p.getStateFrame(), false, true, est_to_drake_func, est_to_drake_func, est_to_drake_func);
      obj.p.getStateFrame.addTransform(trans_drake_to_state_est);
      obj.getStateFrame.addTransform(trans_state_est_to_drake);
      
      % also for the input frames
      trans_u_drake_to_est = AffineTransform(obj.p.getInputFrame(), obj.getInputFrame(), eye(3), zeros(3,1));
      trans_u_est_to_drake = AffineTransform(obj.getInputFrame(), obj.p.getInputFrame(), eye(3), zeros(3,1));
      
      obj.p.getInputFrame.addTransform(trans_u_drake_to_est);
      obj.getInputFrame.addTransform(trans_u_est_to_drake);
      
    end
    
    function [xdot, dxdot] = dynamics(obj, t, x, u)
      options = struct();
      options.grad_method = 'numerical';
      
      if isa(x, 'TaylorVar')
        % no gradients for taylorvar
        xdot = obj.dynamics_no_grad(t, x, u);
        dxdot = [];
        return;
      end
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, dxdot] = geval(tempfunc, t, x, u, options);
    end
    
    function xdot = dynamics_no_grad(obj,t,x,u)
      
      x_drake_frame = ConvertStateEstimatorToDrakeFrame(x);
      x_model_frame = ConvertToModelFrameFromDrakeWorldFrame(x_drake_frame);
      
      xdot_model_frame = tbsc_model(t, x_model_frame, u, obj.p.parameters{:});
      
      xdot = ConvertXdotModelToStateEstimatorFrame(xdot_model_frame);
      
      
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
    
    function playback(obj, xtraj, utraj, options)
      if nargin < 4
        options = struct();
      end
      obj.p.playback(xtraj, utraj, options);
      %error('be super careful, you are attempting playback with a state-estimator-frame plant.  Probably you have plants mixed up.');
      
    end
    
    function playback_xtraj(obj, xtraj, options)
      if nargin < 3
        options = struct();
      end
      
      obj.p.playback_xtraj(xtraj, options);
      %error('be super careful, you are attempting playback with a state-estimator-frame plant.  Probably you have plants mixed up.');
      
    end
      
    
    function v = constructVisualizer(obj)
      v = obj.p.constructVisualizer();
      %error('be super careful, you are attempting playback with a state-estimator-frame plant.  Probably you have plants mixed up.');
    end
    
  end
  
  
end
