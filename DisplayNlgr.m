function DisplayNlgr(states)

  
  for i = 1 : length(states)
    
    %fprintf('\t');
    
    fprintf('  %s', states(i).Name);
    
    for j = 1 : length(states(i).Value)
    
      fprintf('\t% 4.3f', states(i).Value(j));
      
    end
    
    fprintf('\n');
    
  end
  

end