classdef Tree
    methods
        
        
        function obj = Tree(x,y,radius,alignment)
            % Constructor. x and y are points in space. Radius is radius of the
            % tree.  Currently only handles a Plane object.
            %
            % @param x x-position of the tree
            % @param y y-position of the tree
            % @param radius radius of the tree
            %
            % @retval Constructed tree object
            
            if nargin < 4
              alignment = 'vertical'
            end
            
            obj.x = x;
            obj.y = y;
            obj.rad = radius;
            obj.alignment = alignment;
            
        end
        
%         function con = getConstraints(obj)
%             %not sure what this is used for. Left over from
%             %PolygonObstacle2D
%             con.x.c = @(x,y,plane) spaceconstraint(obj, x, y, plane);
%         end
        
        function draw(obj, options)
            % Draws a tree in a 3D space.
            %
            
            if nargin < 2
              options = struct();
            end

            if ~isfield(options, 'sphere_size'), options.sphere_size = 0.05; end
            if ~isfield(options, 'color'), options.color = [0, 0, 1, 0.5]; end
            if ~isfield(options, 'switch_buffers'), options.switch_buffers = true; end
            if ~isfield(options, 'lcmgl_name'), options.lcmgl_name = 'trees'; end

            if ~isfield(options, 'lcmgl')
              lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), options.lcmgl_name);
            else
              lcmgl = options.lcmgl;
            end
            
            checkDependency('lcmgl');
            
            if (length(options.color) < 4)
              options.color(4) = 1;
            end
            
            lcmgl.glColor4f(options.color(1), options.color(2), options.color(3), options.color(4));
            lcmgl.glLineWidth(2);
            
            if strcmp(obj.alignment,'horizontal')
              
                %{
                
                [xs, ys, zs] = cylinder();
                %Scales the cylinder to be 8 units tall, and the appropriate
                %radius.
                zs = zs*(z_high-z_low) + z_low;
                xs = xs*obj.rad + obj.x;
                ys = ys*obj.rad + obj.y;
                
                h = mesh(xs, ys, zs, 'EdgeColor',[0 0 .8],'FaceColor',[0 0 .8]);
                
                % now rotate 90 degrees along the y axis
                rotate(h, [1 0 0], 90);
              
              %}
              
              error('not yet implemented.');
              
              
                
                
            else
                % vertical tree
                
                %{
                
                [xs, ys, zs] = cylinder();
                %Scales the cylinder to be 8 units tall, and the appropriate
                %radius.
                zs = zs*(z_high-z_low) + z_low;
                xs = xs*obj.rad + obj.x;
                ys = ys*obj.rad + obj.y;
                mesh(xs, ys, zs,'EdgeColor',[0 0 .8],'FaceColor',[0 0 .8]);
                
                %}
                
                % draw the obstacle using lcmgl
                %cylinder([x; y; z], r_base, r_top, height, slices, stacks)
                
                lcmgl.cylinder([obj.x; obj.y; 0], obj.rad, obj.rad, 10, 10, 1);
                
            end
            
            if options.switch_buffers
              lcmgl.switchBuffers();
            end
            
            
        end
        
        function dist = ComputeDistance(obj, x, plane)
            % Returns the shortest distance from plane object to the tree and derivatives.
            %
            % This works by projecting the polygon of the Plane onto the XY plane and then
            % finding the distance between that projected 2D polygon and the circle that
            % represents the tree on the XY plane.
            %
            % @param x 12-state vector of the airplane.  Or at a minimum,
            %       x(1) = x, x(2) = y, x(4) = roll
            %       note: there is a singularity at 90 degrees of roll
            %
            % @param plane a Plane object (which defines a bounding box as a polygon)
            %
            % @retval dist distance from the plane to the obstacle
            
            
            planex = x(1);
            planey = x(2);
            planez = x(3);
            roll = x(4);
            pitch = x(5);
            yaw = x(6);
            
            %translate the plane and the tree such that the plane is
            %located at the origin
            if strcmp(obj.alignment,'vertical')
                treex = obj.x-x(1);
                treey = obj.y-x(2);
            else
                treex = obj.x-x(1);
                treez = obj.y-x(3); % This is the shifted z
            end
            
            %Rotation matrix that rotates from Aircraft Body Centered frame
            %to the North-East-Down frame (i.e. projects the aircraft 
            %onto the XY plane if you ignore the new Z components.
            BtoG = [ cos(conj(pitch))*cos(conj(yaw)), cos(conj(yaw))*sin(conj(pitch))*sin(conj(roll)) - cos(conj(roll))*sin(conj(yaw)), sin(conj(roll))*sin(conj(yaw)) + cos(conj(roll))*cos(conj(yaw))*sin(conj(pitch));...
                    cos(conj(pitch))*sin(conj(yaw)), cos(conj(roll))*cos(conj(yaw)) + sin(conj(pitch))*sin(conj(roll))*sin(conj(yaw)), cos(conj(roll))*sin(conj(pitch))*sin(conj(yaw)) - cos(conj(yaw))*sin(conj(roll));...
                     -sin(conj(pitch)),                                                 cos(conj(pitch))*sin(conj(roll)),                                                 cos(conj(pitch))*cos(conj(roll))];
            
            % Project plane onto correct plane based on alignment of obstacle     
            if strcmp(obj.alignment,'vertical')
                rotatedplane = BtoG*[plane.xv; plane.yv; zeros(1, length(plane.yv))];
                [dist, xp, yp] = p_poly_dist(treex,treey,rotatedplane(1,:), rotatedplane(2,:));
            elseif strcmp(obj.alignment,'horizontal')
                rotatedplane = BtoG*[plane.xv; plane.yv; zeros(1,length(plane.yv))];
                [dist, xp, yp] = p_poly_dist(treex,treez,rotatedplane(1,:), rotatedplane(3,:));
            else
                error('This alignment is not supported')
            end
            

            %d was found w.r.t the center of the tree, so we need to
            %subtract the radius of the tree from the distance.
            dist = dist-obj.rad;
            %phi = tanh(-dist);
%             %recover point on rotated plane that is closest to the tree
%             origx = xp;
%             origy = yp;
%             origz = NaN;
%             for i = 1:length(rotatedplane)
%                 %if the point is on the corner of of the rotated, projected
%                 %plane, then its on a corner of the original plane
%                 if (xp == rotatedplane(1,i)) && (yp == rotatedplane(2,i))
%                  origz = rotatedplane(3,i);
%                 else
%                 %if its not a vertex, find which two verticies the point is
%                 %between
%                     if i == length(rotatedplane)
%                         j = 1;
%                     else
%                         j = i+1;
%                     end
%                     x1 = rotatedplane(1,i);
%                     y1 = rotatedplane(2,i);
%                     z1 = rotatedplane(3,i);
%                     x2 = rotatedplane(1,j);
%                     y2 = rotatedplane(2,j);
%                     z2 = rotatedplane(3,j);
%                     %if the point lies on the line between two verticies
%                     %and the point is between the two verticies 
%                     %Uses point-slope form of the line, first checking the
%                     %case if the slope=inf
%                     %(second condition in case plane has 2 separate but
%                     %co-linear segments
%                     if ((x2==x1 && x1 == xp) || (abs((yp-y1) - (((y2-y1)/(x2-x1)) *(xp-x1)))<.001))...
%                         && (abs((x1-xp) + (xp-x2) - (x1-x2)) < .001) && (abs((y1-yp)+(yp-y2)-(y1-y2))<.001)
%                         %find out how far from a vertex the point is, and
%                         %the z-coordinate can be found from this percentage
%                         %and the interpolation between the two vertices
%                         frac = (x1-xp)/(x1-x2); % how far from x1
%                         if isnan(frac)
%                             disp('frac is NaN, trying again');
%                             frac = (y1-yp)/(y1-y2);
%                         end
%                         [x1 y1 z1];
%                         [x2 y2 z2];
%                         origz = z1 - (frac*(z1-z2));%z-coordinate equally far from z1
%                     end
%                     
%                 end
%             end
%             origz;
%             assert(~isnan(origz));
%             dphi = zeros(1,12);
%             %angle from the closest point on the plane to the tree
% 
%             angle = atan2(treey-yp, treex-xp);
%             %xdist and ydist from point on the plane to the closest point
%             %on the tree
%             xdist = treex-xp-(cos(angle)*obj.rad);
%             ydist = treey-yp-(sin(angle)*obj.rad);
%             assert(abs(sqrt(xdist^2+ydist^2) - dist)/dist < (2e-1))
%             
%             %These matricies obtained from symbolic manipulation of the
%             %rotation matricies from the aircraft frame to the ground
%             %frame.  They haven't been tested for accuracy yet.
%             
%             %BtoG * [x;y;z] of the point of the plane, and then
%             %symbolically differentiated.
%             dNEDdroll = [origy*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + origz*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll));...
%                          - origy*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - origz*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw));...
%                          origy*cos(pitch)*cos(roll) - origz*cos(pitch)*sin(roll)];
%             dNEDdpitch = [origz*cos(pitch)*cos(roll)*cos(yaw) - origx*cos(yaw)*sin(pitch) + origy*cos(pitch)*cos(yaw)*sin(roll);...
%                           origz*cos(pitch)*cos(roll)*sin(yaw) - origx*sin(pitch)*sin(yaw) + origy*cos(pitch)*sin(roll)*sin(yaw);...
%                           - origx*cos(pitch) - origz*cos(roll)*sin(pitch) - origy*sin(pitch)*sin(roll)];
%             dNEDdyaw = [origz*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - origy*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - origx*cos(pitch)*sin(yaw);...
%                         origz*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - origy*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + origx*cos(pitch)*cos(yaw);...
%                         0];
%             dDistdroll = (xdist*dNEDdroll(1)/dist + ydist*dNEDdroll(2)/dist);
%             dDistdpitch = (xdist*dNEDdpitch(1)/dist + ydist*dNEDdpitch(2)/dist);
%             dDistdyaw = (xdist*dNEDdyaw(1)/dist + ydist*dNEDdyaw(2)/dist);
%             %d(tanh(dist))/dx = (1-tanh(dist)^2)*d(dist)/d(x)
%             dphi(1) = (1-tanh(-dist)^2)*cos(angle);
%             dphi(2) = (1-tanh(-dist)^2)*sin(angle);
%             %if the tree is not directly in front of the plane
%             % put some thought into this, and then check for accuracy
%             % chain rule: dcost/    = dcost/   * ddist/
%             %             droll       ddist      droll            
%             %check to see if the plane isn't right in front or beside the plane
%             %i.e. if the point on the plane closest to the projection is not a vertex
%             
%             dphi(4) = (1-tanh(-dist)^2)*dDistdroll;
%             dphi(5) = (1-tanh(-dist)^2)*dDistdpitch;
%             dphi(6) = (1-tanh(-dist)^2)*dDistdyaw;
        end
        
    end
    
    properties
        x % X position of the tree
        y % Y position of the tree
        rad % raidus of the tree
        alignment % vertical or horizontal
    end
end
