function cost = get_lqr_K(Q, A, B)

  Q(1) = 1e-10; % ignore x-position
  Q(2) = 1e-10; % ignore y-position
  Q(3) = 1e-10; % ignore z-position



  R = diag([25 25 25]);
  
  Q
  
  Q = diag(Q);


  K = lqr(full(A), full(B), Q, R);


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

  
  K(:,1:3) = 0;
  K(:, 6:9) = 0;
  K(:, 12) = 0;
  
  K_pd(3,:) = 0;
  K(3,:) = 0;
  
  K
  

  cost = norm(K(1,4:5)-K_pd(1,4:5), 1) + 15*norm(K(1,10:11) - K_pd(1,10:11), 1);
  
  cost




end