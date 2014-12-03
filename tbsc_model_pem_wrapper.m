function [ xdot, y_pem ]= tbsc_model_pem_wrapper(t,x,u, elev_lift_fac, elev_drag_fac, varargin)

  [xdot, y] = tbsc_model(t, x, u, elev_lift_fac, elev_drag_fac, varargin);
  
  y_pem = [ xdot(4); xdot(5); xdot(6) ];
  
  

end