function dist = ComputeTotalDistance(t_start, t_end, est)

  est_trim = TrimEst(t_start, t_end, est);
  
  dist = 0;
  
  x = est_trim.pos.x(1);
  y = est_trim.pos.y(1);
  z = est_trim.pos.z(1);
  
  for i = 2 : length(est_trim.logtime)
    
    delta_x = est_trim.pos.x(i) - x;
    delta_y = est_trim.pos.y(i) - y;
    delta_z = est_trim.pos.z(i) - z;
    
    dist = dist + sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
    
    x = est_trim.pos.x(i);
    y = est_trim.pos.y(i);
    z = est_trim.pos.z(i);
    
    
  end
    


end