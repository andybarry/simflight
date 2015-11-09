function SaveComparison(name_str, fig_num, subplot_num)
  figure(fig_num)
  plt = subplot(3,1,subplot_num);
  hfig = figure(10);
  clf
  plot_new = copyobj(plt, hfig);
  set(plot_new, 'Position', get(0, 'DefaultAxesPosition'));
  title('')
  legend('Actual','Planned');
  saveasAll(['figures/' name_str], 17);
end