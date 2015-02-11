
clear

bounds = [ 2
  .01
  .01
  .5
  .5
  .5
    .5
    3
    3
  1
  1
  1];

u0 = [0, 0, 5.33/2];




xf_straight = [7.1738
         0
   0
         0
    0.1857
         0
   15.0805
         0
   0
         0
    0
         0];
       
tf_straight = 0.5;


[straight_utraj, straight_xtraj] = runDircol(xf_straight, tf_straight, bounds, u0);


%{

bounds = [ 10
  .5
  1
  .5
  .5
  .5
    3
    5
    5
  deg2rad(70)
  deg2rad(70)
  deg2rad(70)];



xf_left = [10
         3
   2
         0
    0.1857
         0
   15.0805
         2
   0
         deg2rad(30)
    0
         0];
       
tf_left = 0.5;

[left_utrag, left_xtraj] = runDircol(xf_left, tf_left, bounds, u0);

%}
%{
xf_right = [5
         -2.5
   2
         0
    0.1857
         0
   15.0805
         -2
   0
         -deg2rad(30)
    0
         0];
       
tf_right = 0.5;

[right_utrag, right_xtraj] = runDircol(xf_right, tf_right, bounds, u0);
%}
