function [rad_to_servo, servo_to_rad, minmaxtrim] = ReadSimpleConfigServos(filename)
  % Reads a .cfg file for bot-param-server and looks for particular fields
  disp(['Reading ' filename ' for config info']);

  
  rad_to_servo = struct();
  servo_to_rad = struct();
  minmaxtrim = struct();
  trim = struct();

  current_obj = 1;
  
  file_id = fopen(filename,'r');
  
  tline = fgetl(file_id);
  while ischar(tline)
    
    if ~isempty(strfind(tline, 'radians_to_servo'))
      current_obj = 1;
    elseif ~isempty(strfind(tline, 'servo_to_radians'))
      current_obj = 2;
    end

    value = ReadKeyLine(tline, 'elevL_slope');
    if ~isempty(value)
      if current_obj == 1
        rad_to_servo.elevL_slope = value;
      else
        servo_to_rad.elevL_slope = value;
      end
    end
    
    value = ReadKeyLine(tline, 'elevL_y_intercept');
    if ~isempty(value)
      if current_obj == 1
        rad_to_servo.elevL_y_intercept = value;
      else
        servo_to_rad.elevL_y_intercept = value;
      end
    end
    
    value = ReadKeyLine(tline, 'elevR_slope');
    if ~isempty(value)
      if current_obj == 1
        rad_to_servo.elevR_slope = value;
      else
        servo_to_rad.elevR_slope = value;
      end
    end
    
    value = ReadKeyLine(tline, 'elevR_y_intercept');
   if ~isempty(value)
      if current_obj == 1
        rad_to_servo.elevR_y_intercept = value;
      else
        servo_to_rad.elevR_y_intercept = value;
      end
    end
    
    value = ReadKeyLine(tline, 'throttle_slope');
    if ~isempty(value)
      if current_obj == 1
        rad_to_servo.throttle_slope = value;
      else
        servo_to_rad.throttle_slope = value;
      end
    end
    
    value = ReadKeyLine(tline, 'throttle_y_intercept');
    if ~isempty(value)
      if current_obj == 1
        rad_to_servo.throttle_y_intercept = value;
      else
        servo_to_rad.throttle_y_intercept = value;
      end
    end
    
    value = ReadKeyLine(tline, 'elevL_min');
    if ~isempty(value)
      minmaxtrim.elevL_min = value;
    end
    
    value = ReadKeyLine(tline, 'elevL_max');
    if ~isempty(value)
      minmaxtrim.elevL_max = value;
    end
    
    value = ReadKeyLine(tline, 'elevR_min');
    if ~isempty(value)
      minmaxtrim.elevR_min = value;
    end
    
    value = ReadKeyLine(tline, 'elevR_max');
    if ~isempty(value)
      minmaxtrim.elevR_max = value;
    end
    
    value = ReadKeyLine(tline, 'throttle_min');
    if ~isempty(value)
      minmaxtrim.throttle_min = value;
    end
    
    value = ReadKeyLine(tline, 'throttle_max');
    if ~isempty(value)
      minmaxtrim.throttle_max = value;
    end
    
    value = ReadKeyLine(tline, 'elevL_trim');
    if ~isempty(value)
      minmaxtrim.elevL_trim = value;
    end
    
    value = ReadKeyLine(tline, 'elevR_trim');
    if ~isempty(value)
      minmaxtrim.elevR_trim = value;
    end
    
    value = ReadKeyLine(tline, 'throttle_trim');
    if ~isempty(value)
      minmaxtrim.throttle_trim = value;
    end
    
    value = ReadKeyLine(tline, 'elevL_flight_trim');
    if ~isempty(value)
      minmaxtrim.elevL_flight_trim = value;
    end
    
    value = ReadKeyLine(tline, 'elevR_flight_trim');
    if ~isempty(value)
      minmaxtrim.elevR_flight_trim = value;
    end
    
    
    %disp(tline)
    tline = fgetl(file_id);
  end

  
  
  fclose(file_id);

end


function value = ReadKeyLine(line, key)
  
  % handle comments
  comment_loc = strfind(line, '#');
  if ~isempty(comment_loc)
    line = line(1:comment_loc - 1);
  end

  str_loc = strfind(line, key);
  if ~isempty(str_loc)
    eq_loc = strfind(line, '=');
    if ~isempty(eq_loc)
      str_value = line(eq_loc+1:end);
      str_value = strrep(str_value, ';', '');
      value = str2double( str_value );
      
      if isnan(value)
        error(['Read NaN value on line: "' line '" when looking for key: "' key '"']);
      end
    end
  else
    value = [];
  end

end