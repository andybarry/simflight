function [ xdot, y_pem ] = tbsc_model_pem_wrapper(t,x,u, elev_lift_fac, elev_drag_fac, M_P_fac, M_Q_fac, M_R_fac, varargin)

  [xdot, y] = tbsc_model(t,x,u, elev_lift_fac, elev_drag_fac, M_P_fac, M_Q_fac, M_R_fac, varargin);
  
  %y_pem = [ y(4); y(5); y(6) ];
  
  y_pem = [ y(4); y(5); y(6); y(7) ];
  
  

end