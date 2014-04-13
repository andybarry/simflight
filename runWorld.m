
checkDependency('lcmgl');


% init rigid body manipulator

options.floating = true;
options.terrain = [];


rb = RigidBodyManipulator([getDrakePath '/systems/plants/test/FallingBrick.urdf'], options);


% world is rb.body(1)

world = rb.getBody(1);


% add obstacles to the world

obstacle1 = RigidBodyCylinder(1, 10);
obstacle1.T(1:3, 4) = [ 10; 0; 0 ];

world.contact_shapes{end+1} = obstacle1;
world.visual_shapes{end+1} = obstacle1;

obstacle2 = RigidBodyCylinder(1, 10);
obstacle2.T(1:3, 4) = [ -10; 0; 0 ];

world.contact_shapes{end+1} = obstacle2;
world.visual_shapes{end+1} = obstacle2;


% put the new world back into the rb
rb = rb.setBody(1, world);
rb = compile(rb);



zheight = 2;


q0 = rand(rb.getNumDOF(), 1);
q0(2) = 10;
q0(3) = zheight;

x0 = [q0; rand(rb.getNumDOF(), 1)]; % full state (positions, velocities)

v = rb.constructVisualizer();
v.draw(0, x0);


kinsol = doKinematics(rb, q0);

[phi, ~, closestPointsA, closestPointsB, bodyIndexA, bodyIndexB] = rb.collisionDetect(kinsol);


% closest points are in body frame, we want them in world frame
xA_in_world = zeros(size(closestPointsA));
xB_in_world = zeros(size(closestPointsB));
for i = 1:rb.getNumBodies()
  xA_in_world(:,bodyIndexA==i) = forwardKin(rb,kinsol,i,closestPointsA(:,bodyIndexA==i),0);
  xB_in_world(:,bodyIndexB==i) = forwardKin(rb,kinsol,i,closestPointsB(:,bodyIndexB==i),0);
end


lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'collision');

lcmgl.glColor3f(1, 0, 0);
for point = xA_in_world
    lcmgl.sphere(point, .5, 20, 20);
end
lcmgl.switchBuffers;

% test raycasting

distance = [];
angles = [];



for angle = 0:.01:2*pi
    
    origin = [0; 0; zheight];
    
    point_on_ray = [cos(angle); sin(angle); 0] * 1000;
    point_on_ray(3) = zheight;
    

    
    angles(end+1) = angle;
    distance(end+1) = collisionRaycast(rb, kinsol, origin, point_on_ray);
    
end

lcmgl.glColor3f(1, 0, 0);
for i=1:length(distance)
    if (distance(i) > 0)
        
        point = [cos(angles(i)); sin(angles(i)); 0] * distance(i);
        point(3) = zheight;
        
        
        lcmgl.sphere(point, .1, 20, 20);
        lcmgl.line3(origin(1), origin(2), origin(3), point(1), point(2), point(3));
        
    end
end
lcmgl.switchBuffers;
