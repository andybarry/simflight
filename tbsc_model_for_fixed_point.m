function xdot_small = tbsc_model_for_fixed_point(x, u, x_drag)

  parameters = {0.9040, 0, -0.1340, -0.0490, 0, 0.0300};
  
  xdot = tbsc_model(0, x, u, parameters{1:end-1}, x_drag);
  
  xdot_small = xdot(7:12);
  
  xdot_small = norm(xdot_small);
end