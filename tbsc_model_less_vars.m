function xdot = tbsc_model_less_vars(small_state, u, parameters)


  full_state = zeros(12,1);
  
  full_state(5) = small_state(1);
  full_state(7) = small_state(2);
  
  
  full_state = ConvertToModelFrameFromDrakeWorldFrame(full_state);
      
  xdot_model_frame = tbsc_model(0, full_state, u, parameters{:});

  xdot_full = ConvertXdotModelToDrake(full_state, xdot_model_frame);
  

  xdot = xdot_full(7:12);
  
end