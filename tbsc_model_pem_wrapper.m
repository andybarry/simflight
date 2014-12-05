function [ xdot, y_pem ]= tbsc_model_pem_wrapper(t,x,u, Jx_fac, Jy_fac, Jz_fac, varargin)

  [xdot, y] = tbsc_model(t, x, u, Jx_fac, Jy_fac, Jz_fac, varargin);
  
  y_pem = [ y(4); y(5); y(6) ];
  
  

end