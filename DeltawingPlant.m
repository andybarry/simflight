classdef DeltawingPlant < DrakeSystem
% Defines the dynamics for the TBS Capi delta wing.

  properties 

  end
  
  methods
    function obj = DeltawingPlant()
      obj = obj@DrakeSystem(12,0,3,12,false,true);
      obj = setDirectFeedthrough(obj,0);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
    
    function xdot = dynamics(obj,t,x,u)
      
      x = ConvertToModelFrameFromDrakeWorldFrame(x);
      
      xdot = tbsc_model(t, x, u, 1);
      
      % tbsc_model outputs in drake frame
      
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
