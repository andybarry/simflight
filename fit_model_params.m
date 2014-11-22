% % Break into pieces
% numPieces = 2;
% pieces_length = floor(length(outputs)/numPieces);
% z = {};
% for k = 1:numPieces
%     torque_pieces{k} = torque((k-1)*pieces_length+1:k*pieces_length)';
%     outputs_pieces{k} = outputs(:,(k-1)*pieces_length+1:k*pieces_length)';
%     z{end+1} = iddata(outputs_pieces{k},torque_pieces{k},dt);
% end

load dataWithTop.mat
% Merge data we want to fit
%z = merge(z1,z2,z3,z4,z7);
z = merge(z1,z2,z3,z4,zTraj);


FileName = 'AcrobotModel';
Order = [2, 1, 4];
%Parameters = [2.178;0.45;0.5;0.15;0.37;0.05;0.1;0.2091;0.0942]; % m1,m2,l1,lc1,lc2,b1,b2,I1,I2 (elena's params)
%Parameters = [2.1503;0.9137;0.4619;0.4709;0.5802;0;0.1109;0.5178;0.2561]; % New sysid params (thing that worked with lqr)
Parameters =     [2.1512;
    0.9077;
    0.5019;
    0.4753;
    0.5528;
    0.2644;
    0.0529;
    0.5324;
    0.2376];

InitialStates = [0;0;0;0];
Ts = 0;

nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts);

setinit(nlgr, 'Fixed', {false false false false});   % Estimate the initial state.
nlgr = pem(z, nlgr, 'Display', 'Full','MaxIter',100);

figure;
compare(z, nlgr);

% % Test on other data
% ztest = merge(z3,z4,z5); %iddata(outputs2,torque2,dt);
% figure;
% compare(ztest,nlgr);