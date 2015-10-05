function [parameters, gains, gains_ti] = GetDefaultGains()

  %parameters = {0.904, 0.000, -0.134, -0.049, 0 };
  %parameters = {0.904, 0.000, -0.035, -0.091, 0 };
  
  % HAD THESE FOR A LONG TIME (SUMMER 2015)
  %parameters = {1.027, 0.001, -0.094, -0.115, 0 };
  
  % SUUCESSFUL ON 09/08/2015
  %parameters = {1.027, 0.001, -0.074, -0.115, 0 };
  
  
  %parameters = {0.727, 0.616, 0.237, -0.092, -3.357 };
  %parameters = {0.727, 0.616, 0.237, -0.092, 0 };
  
  % not great 9/18/2015, overrolls
  %parameters = {0.223, 0.001, -0.065, -0.039, -0.005 };
  
  % produced a very stable TI roll, but was supposed to be 11 deg and was
  % 50 deg
  % 9/28/15
  parameters = {1.3, 0.101, -0.074, -0.115, 0 };
  
  % pretty bad
  %parameters = {1.027, 0.001, -0.054, -0.115, 0 };

  % WORKING WELL 
  %Q = diag([10 10 10 10 50 .25 0.1 .0001 0.0001 .1 .01 .1]);
  %Q(1,1) = 1e-10; % ignore x-position
  %Q(2,2) = 1e-10; % ignore y-position
  %Q(3,3) = 1e-10; % ignore z-position 
  
  
  % EXPERIMENTAL FOR XYZ GAINS
  % WORKED FOR SUCCESS ON 09/08/2015
  Q = diag([10 10 10 5 30 .25 0.1 .0001 0.0001 .1 .05 .1]);
  Q_ti = Q;
  Q_ti(1,1) = 1e-10;
  Q_ti(2,2) = 1;
  Q_ti(3,3) = 1;
  
  
   
  % Q = diag([10 10 10 10 50 .25 0.1 0.1 0.1 .1 .01 .1]);
%  Q = diag([100*ones(6,1);10*ones(6,1)]);
%  Qf = Q;
   

    %R_values = [150 100];
    %R_values = [150, 200];
    
    % USED ON SEPT 8 SUCCESS RUN
    %R_values = [200, 400];
    
    R_values = [200];
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
  
  % THESE ARE GOOD ENOUGH TO GRADUATE
%   % y, P
%   K_pd_xyz(1,2) = 0.1;
%   K_pd_xyz(2,2) = -0.1;
%   
%   % z, P
%   K_pd_xyz(1,3) = 0.25;
%   K_pd_xyz(2,3) = 0.25;

  K_pd_xyz(1,2) = 0.2;
  K_pd_xyz(2,2) = -0.2;
  
  % z, P
  K_pd_xyz(1,3) = 0.3;
  K_pd_xyz(2,3) = 0.3;

  % y, P
  % BAD over roll K_pd_xyz(1,2) = 0.75;
  % BAD over roll K_pd_xyz(2,2) = -0.75;
  
  % z, P
  % BAD over roll K_pd_xyz(1,3) = 1.0;
  % BAD over roll K_pd_xyz(2,3) = 1.0;
  

  gains.Q = Q;
  gains.Qf = Qf;
  gains.R_values = R_values;
  gains.K_pd = K_pd;
  gains.K_pd_xyz = K_pd_xyz;
  
  gains_ti = gains;
  gains_ti.Q = Q_ti;
  gains_ti.R_values = [gains.R_values];


end