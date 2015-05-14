function gains = GetDefaultGains()


    Q = diag([0 0 0 10 50 .25 0.1 .0001 0.0001 .1 .01 .1]);
    Q(1,1) = 1e-10; % ignore x-position
    Q(2,2) = 1e-10; % ignore y-position
    Q(3,3) = 1e-10; % ignore z-position

    R_values = [150 100];

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

    K_pd_yaw = K_pd;
    K_pd_aggressive_yaw = K_pd;

    K_pd_yaw(1,6) = 0.25;
    K_pd_yaw(2,6) = -0.25;

    K_pd_aggressive_yaw(1,6) = 0.5;
    K_pd_aggressive_yaw(2,6) = -0.5;

    gains.Q = Q;
    gains.Qf = Qf;
    gains.R_values = R_values;
    gains.K_pd = K_pd;
    gains.K_pd_yaw = K_pd_yaw;
    gains.K_pd_aggressive_yaw = K_pd_aggressive_yaw;


end