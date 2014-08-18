function chord_squared = tbsc_chord_length_squared(y)
  % Returns the chord length squared as a function of position from the left
  % wingtip
  %
  % Only valid for the left half of the wing
  %
  % @param y length along wing in inches
  %
  % @retval instantaneous chord length in inches squared

  

  
  if (length(y) > 1)
    
    count = 1;
    for this_y = y
      chord(count) = tbsc_chord_length_helper(this_y);
      count = count + 1;
    end
    
    
  else
    chord = tbsc_chord_length_helper(y);
    
  end
  
  chord_squared = chord .* chord;
  
  function chord = tbsc_chord_length_helper(y)
    
    theta = 0.201;
    psi = 0.522403;
    gamma = 1.21203;

    if (y < 0)
      error('y must be greater than 0.');
    end

    if ( y <= 12.375 )


      chord = 12 - ( tan(psi) * ( 12.375 - y) ) - y * tan(theta);

    elseif ( y <= 15.5 )

      c = 0;

      if ( y <= 12.975 )
        c = tan(gamma) * ( 0.6 - ( y - 12.375 ) );
      end

      d = tan(psi) * ( y - 12.375 );

      chord = c + d + 7.9;

    elseif ( y <= 17 )

      chord = 9.6992;

    else
      error('y exceeds wing length');
    end
  end
  
end