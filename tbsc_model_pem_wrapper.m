function [ xdot, y_pem ]= tbsc_model_pem_wrapper(t,x,u, Jx_fac, Jy_fac, Jz_fac, elev_lift_fac, elev_drag_fac, varargin)

  [xdot, y] = tbsc_model(t, x, u, Jx_fac, Jy_fac, Jz_fac, elev_lift_fac, elev_drag_fac, varargin);
  
  y_pem = [ xdot(4); xdot(5); xdot(6) ];
  
  

end