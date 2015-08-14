function DrawPointsLcmGl(points)
  lcmgl = LCMGLClient('points');
  lcmgl.glColor3f(0,0,1);
  for i = 1 : size(points,2)
    lcmgl.sphere(points(:, i), 0.25, 20, 20);
  end

  lcmgl.switchBuffers();
end
