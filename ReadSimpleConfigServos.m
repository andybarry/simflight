function [servo_to_rad, rad_to_servo] = ReadSimpleConfigServos(filename)
  % Reads a .cfg file for bot-param-server and looks for particular fields

  rad_to_servo = struct();
  servo_to_rad = struct();
  minmax = struct();

  current_obj = rad_to_servo;
  
  file_id = fopen(filename,'r');
  
  tline = fgetl(file_id);
  while ischar(tline)
    
    if strfind(tline, 'radians_to_servo')
      current_obj = rad_to_servo;
    elseif strfind(tline, 'servo_to_radians')
      current_obj = servo_to_rad;
    end

    
    
    disp(tline)
    tline = fgetl(file_id);
  end

  
  
  fclose(file_id);

end