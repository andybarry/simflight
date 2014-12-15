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
    
    function xdot = dynamics(obj,t,x,u)
      
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
  
end
