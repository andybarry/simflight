function DrawLinesAtTimes(times, colorstr)

  
  
  % check for subplot
  axes = findall(gcf, 'type', 'axes');
  num_plots = length(axes);
  for q = 1 : num_plots
    
    if (num_plots > 1)
      subplot(num_plots, 1, q);
      hold on
    end
    
    xlim_vals = get(gca, 'XLim');
    ylim_vals = get(gca, 'YLim');
    
    hold on;
    
    for i = 1 : length(times)
      plot([times(i); times(i)], ylim_vals, colorstr);
    end
    
    %xlim(xlim_vals);
    %ylim(ylim_vals);
    
  end

  
  

end