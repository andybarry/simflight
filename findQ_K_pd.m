
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


% use fmincon to find a Q that matches K_pd as close as possible



%Q0 = [0 0 0 1 1 1 1 1 1 1 1 1];
Q0 = 20*rand(1, 12);
Q0(1) = 1e-10; % ignore x-position
Q0(2) = 1e-10; % ignore y-position
Q0(3) = 1e-10; % ignore z-position


A_fmincon = -eye(12);
b_fmincon = -1e-5 * ones(12,1);

options = optimoptions(@fmincon,'Display','iter','MaxFunEvals',100000);

myfun = @(Q) get_lqr_K(Q, A, B);

x = fmincon(myfun, Q0, A_fmincon, b_fmincon, [], [], [], [], [], options);
