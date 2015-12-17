
%%
% _You may have noticed some recent changes in the format of this blog. Hereâ€™s
% what to expect on a regular basis â€“ two topics per week._ 
%%
% * _On Tuesdays Doug will provide MATLAB tutorials._
% * _On Fridays guest bloggers Jiro, Brett and Bob will highlight File
% Exchange submissions._

%%
% A file need not be long to be useful. <http://www.mathworks.com/matlabcentral/fileexchange/loadAuthor.do?objectId=1093599&objectType=author Brett>'s pick this week, <http://www.mathworks.com/matlabcentral/fileexchange/loadAuthor.do?objectId=1093629&objectType=author Izhak Bucher>'s <http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=5557&objectType=FILE Circle Fit>, is only 5 lines long, excluding comments. But I really like Izhak's entry, and have had many opportunities to use it in the several years since I downloaded it.
% Somehow, the requirement of fitting a circle to some points seems to occur with puzzling frequency in my work. Izhak's CIRCFIT makes it easy, and is quite robust in most cases.

%traj = lib.GetTrajectoryByNumber(5);
%[~, idx1] = min(abs(x(1,:) - 5.589));
%[~, idx2] = min(abs(x(1,:) - 9.84));

traj = lib.GetTrajectoryByNumber(6);
t = 0:0.01:traj.xtraj.tspan(2);

x = traj.xtraj.eval(t);
plot(x(1,:), x(2,:))

[~, idx1] = min(abs(x(1,:) - 7.63));
[~, idx2] = min(abs(x(1,:) - 8.643));

% [~, idx1] = min(abs(x(1,:) - 21.21));
% [~, idx2] = min(abs(x(1,:) - 29.57));

xs = x(1,idx1:idx2);
ys = x(2, idx1:idx2);


%%
% So how do you use Izhak's function? Consider these noisy data created for
% illustrative purposes:

% R = 10; x_c = 5; y_c = 8;
% thetas = 0:pi/64:pi;
% xs = x_c + R*cos(thetas);
% ys = y_c + R*sin(thetas);
% 
% % Now add some random noise to make the problem a bit more challenging:
% mult = 0.5;
% xs = xs+mult*randn(size(xs));
% ys = ys+mult*randn(size(ys));

%%
% Plot the points...
figure
plot(xs,ys,'b.')
axis equal

%%
% ...and then calculate and display the best-fit circle
[xfit,yfit,Rfit] = circfit(xs,ys);
figure
plot(xs,ys,'b.')
hold on
rectangle('position',[xfit-Rfit,yfit-Rfit,Rfit*2,Rfit*2],...
    'curvature',[1,1],'linestyle','-','edgecolor','r');
title(sprintf('Best fit: R = %0.1f; Ctr = (%0.1f,%0.1f)',...
    Rfit,xfit,yfit));
plot(xfit,yfit,'g.')
xlim([xfit-Rfit-2,xfit+Rfit+2])
ylim([yfit-Rfit-2,yfit+Rfit+2])
axis equal

%% 
% By the way, if it seems odd to you to use a RECTANGLE command to plot a
% circle, stay tuned for next week's Pick of the Week!
%%
% What other data-fitting problems do you commonly come up against, and how
% do you solve them?

%%
% _Brett Shoelson_
% _Copyright 2008 The MathWorks, Inc._

