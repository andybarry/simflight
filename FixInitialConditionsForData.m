function x0_dat_full = FixInitialConditionsForData(data)

  if iscell(data.OutputData)

    for i = 1 : length(data.OutputData)

      x0_dat{i} = data.OutputData{i}(1,:);

    end
    num_data = length(data.OutputData);
  else
    x0_dat{1} = data.OutputData(1,:);
    num_data = 1;
  end

  x0_dat_full{1} = zeros( 1, num_data);
  x0_dat_full{2} = zeros( 1, num_data);
  x0_dat_full{3} = zeros( 1, num_data);

  x0_dat_full{4} = [];
  x0_dat_full{5} = [];
  x0_dat_full{6} = [];
  x0_dat_full{7} = [];

  for i = 1 : length(x0_dat)
    x0_dat_full{4} = [ x0_dat_full{4} x0_dat{i}(1) ];
    x0_dat_full{5} = [ x0_dat_full{5} x0_dat{i}(2) ];
    x0_dat_full{6} = [ x0_dat_full{6} x0_dat{i}(3) ];
    x0_dat_full{7} = [ x0_dat_full{7} x0_dat{i}(4) ];
  end

  %x0_dat_full{7} = [ 9.9607 11.3508 ];%zeros( 1, length(merged_dat.OutputData));

  x0_dat_full{8} = zeros( 1, num_data);
  x0_dat_full{9} = zeros( 1, num_data);

  x0_dat_full{10} = zeros( 1, num_data);
  x0_dat_full{11} = zeros( 1, num_data);
  x0_dat_full{12} = zeros( 1, num_data);

end
