t = 0:0.01:3;

for i = 1 : length(t)
  x_drake(i,:) = ConvertDrakeFrameToEstimatorFrame(xtraj_knife1.eval(t(i)));
end

x_est = test.eval(t);

figure(1)
clf
plot(x_drake)
hold all
plot(x_est')