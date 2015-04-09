function ChangeFontSizes(fontsize, axis_array)
    % Changes the font sizes on the current figure
    %
    % Font size not changing?
    %   Try: sudo apt-get install xfonts-100dpi xfonts-75dpi and then
    %   logging out and back in.
    %
    % @param fontsize (optional) specify the plot's font size (often useful
    %   for embedding in papers
    % @param (Optional) axis_array array of axes to change the font size on
    %   useful for multi-axis plots (such as plotyy)
      
      if (nargin < 2)
          axis_array(1) = gca;
      end

      hlegend = findobj(gcf,'Type','axes','Tag','legend');

      if (exist('hlegned'))
          set(hlegend,'FontSize', fontsize) 
      end

      for i=1:length(axis_array)
          xlhand = get(axis_array(i),'xlabel');
          set(xlhand, 'fontsize', fontsize);

          ylhand = get(axis_array(i),'ylabel');
          set(ylhand, 'fontsize', fontsize);

          zlhand = get(axis_array(i),'zlabel');
          set(zlhand, 'fontsize', fontsize);

          thand = get(axis_array(i),'title');
          set(thand, 'fontsize', fontsize);

          set(axis_array(i), 'FontSize', fontsize);
      end

end
