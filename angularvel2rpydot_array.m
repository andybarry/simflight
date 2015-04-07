function rpydot_array = angularvel2rpydot_array(angular_x, angular_y, angular_z)

  assert(length(angular_x) == length(angular_y));
  assert(length(angular_x) == length(angular_z));

  rpydot_array = zeros(length(angular_x), 3);
  
  for i = 1 : length(angular_x)
    
    rpydot_array(i,:) = angularvel2rpydot([angular_x(i), angular_y(i), angular_z(i)]);
    
  end


end