

count = 1;

y_vals = 0:0.01:17;
for y = y_vals
  
  chord(count) = tbsc_chord_length(y);
  count = count + 1;
  
end


plot(y_vals, chord','x-');


chord2 = tbsc_chord_length(y_vals);





S = 254.7;
b = 17*2;

mean_aerodynamic_chord = 2 / S  * integral(@tbsc_chord_length_squared, 0, b/2)

% >> computeChord
% 
% mean_aerodynamic_chord =
% 
%     8.2559