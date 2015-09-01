function [parameters, gains] = GetDefaultGains()

  %parameters = {0.904, 0.000, -0.134, -0.049, 0 };
  %parameters = {0.904, 0.000, -0.035, -0.091, 0 };
  parameters = {1.027, 0.001, -0.094, -0.115, 0 };

  % WORKING WELL 
  %Q = diag([10 10 10 10 50 .25 0.1 .0001 0.0001 .1 .01 .1]);
  %Q(1,1) = 1e-10; % ignore x-position
  %Q(2,2) = 1e-10; % ignore y-position
  %Q(3,3) = 1e-10; % ignore z-position 
  
  
  % EXPERIMENTAL FOR XYZ GAINS
  Q = diag([10 10 10 5 30 .25 0.1 .0001 0.0001 .1 .05 .1]);
   
   
   
  % Q = diag([10 10 10 10 50 .25 0.1 0.1 0.1 .1 .01 .1]);
%  Q = diag([100*ones(6,1);10*ones(6,1)]);
%  Qf = Q;
   

    %R_values = [150 100];
    %R_values = [150, 200];
    R_values = [200, 150];
  %R_values = 0.5;

   Qf = eye(12);


  K_pd = zeros(3,12);
  

  % roll P
  K_pd(1,4) = -0.4;
  K_pd(2,4) = 0.4;

  % roll D
  K_pd(1,10) = -0.02;
  K_pd(2,10) = 0.02;

  % pitch P
  K_pd(1,5) = -0.4;
  K_pd(2,5) = -0.4;

  % pitch D
  K_pd(1,11) = -0.02;
  K_pd(2,11) = -0.02;

  
  K_pd_xyz = K_pd;
  % x, P -- still zeros
  
  % y, P
  K_pd_xyz(1,2) = 0.1;
  K_pd_xyz(2,2) = -0.1;
  
  % z, P
  K_pd_xyz(1,3) = 0.25;
  K_pd_xyz(2,3) = 0.25;
  

  gains.Q = Q;
  gains.Qf = Qf;
  gains.R_values = R_values;
  gains.K_pd = K_pd;
  gains.K_pd_xyz = K_pd_xyz;


end