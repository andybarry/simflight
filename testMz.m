%% state

pos = [643; -19.59; -14.06];

UVW = [12.358; -0.37176; -7.321641];

q = [ 0.92042916327673; 0.075662452847708837; -0.3131459922391228; -0.22142871003294068];

PQR = [ -0.00431; 0.02377; 1.199e-4];

rpy = quat2rpy(q);

disp('rpy in deg: ');
disp(rad2deg(rpy));

x_state_est = [pos; rpy; UVW; PQR];

x_drake = ConvertStateEstimatorToDrakeFrame(x_state_est);


%% extract yaw

yaw = rpy(3);

Mz = rotz(-yaw);

pos2 = Mz * pos;

rot_mat2 = rpy2rotmat(rpy);

rpy2 = rotmat2rpy( Mz * rot_mat2 );

R_body_to_world = rpy2rotmat(rpy);

vel2 = Mz * R_body_to_world * UVW;

pqr2 = Mz * R_body_to_world * PQR;

rpydot = angularvel2rpydot(rpy2, pqr2);

x_drake2 = [pos2; rpy2; vel2; rpydot];

%x_state_est - TrajectoryInLibrary.ConvertXVectorToEstimatorFrame(x_drake)

x_state_est2 = ConvertDrakeFrameToEstimatorFrame(x_drake2)

x_state_est2 - x_state_est


disp('output:')
x_drake2
