function f = tbsc_model_wrapper(x_in, u_in, measured_values, elev_lift_fac)

  % call the model for each run of the data
  
  f = zeros(length(x_in), 1);
  
  
  for i = 1 : length(x_in)
    xdot = tbsc_model(0, x_in(i,:), u_in(i,:), elev_lift_fac);
    
    f(i) = norm(xdot(7:12)' - measured_values(i,:));
    
  end
  fprintf('.');

end