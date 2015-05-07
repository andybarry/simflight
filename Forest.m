classdef Forest
    %Holds many Trees, and can operate on them, mainly useful for only
    %Trees
    %Analogous to ObstacleField in the 2-D case
    properties
        obstacles; % Cell array of Tree Instances (currently, unless expanded)
        number_of_obstacles;
        plane;
    end
    
    methods
        function obj = Forest()
            obj.number_of_obstacles = 0;
            %if nargin>0
            %    obj.plane = plane;
            %end
        end
        function obj = AddPlane(obj, p)
            %adds a Plane to the forest
            obj.plane = p;
        end

        function obj = TheTwoTowers(obj, width, x)
            % Generates two trees directly ahead, at  x = 5, y = +-1
            %
            % @param width The distance each tree is from the y = 0.  A width of 1 puts the center of the trees 2 units apart.
            % @param x The x-distance ahead to the center of the obstacles.
            %
            % @retval obj the new forest
            if nargin<2
                width = 1;
            end
            if nargin<3
                x = 5;
            end
            obj = obj.AddTree(Tree(x,-width,.1));
            obj = obj.AddTree(Tree(x,width,.1));
        end
        
        
        function draw(obj)
            %Draws the Forest. Implemented by calling draw() on each Tree
            %{
            persistent hFig;
            if (isempty(hFig))
                hFig = sfigure(25);
                set(hFig,'DoubleBuffer', 'on');
                set(gca,'YDir', 'reverse');
                set(gca,'ZDir', 'reverse');
            end
            hold on
            for i = 1:obj.number_of_obstacles
                currTree = obj.obstacles{i};
                currTree.draw();
            end
            %}
            
            lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'obstacles');
            options = struct();
            options.lcmgl = lcmgl;
            options.switch_buffers = false;
            
            for i = 1:obj.number_of_obstacles
                currTree = obj.obstacles{i};
                currTree.draw(options);
            end
            
            lcmgl.switchBuffers();
            
            
        end
        
            
        
        function con = AddConstraints(obj,con,plane)
           % Adds the con.x.c constraint for the Forest to the given
           % constraint object.
           %
           % @param con input constraint object.  Must be a structure.
           %    Other than that, we'll just overwrite con.x.c, so anything
           %    else will be returned unchanged.
           % @param plane An object of Plane
           %
           % @retval con Constraint structure with modified con.x.c value.
            if nargin>2
                obj.plane = plane;
            end
            con.x.c = @(x)obj.obstacleConstraint(x);
        end
        
        function obj = AddTree(obj, tree, index, alignment)
            %Adds a tree to the obstacle field
            %also allows replacing a tree with a new one if an index is
            %given
            if nargin<3
                index = obj.number_of_obstacles + 1;
                obj.number_of_obstacles = obj.number_of_obstacles + 1;
            elseif index<0
                index = obj.number_of_obstacles + 1;
                obj.number_of_obstacles = obj.number_of_obstacles + 1;
            elseif index>(obj.number_of_obstacles + 1)
                index = obj.number_of_obstacles + 1;
                obj.number_of_obstacles = obj.number_of_obstacles + 1;
            end
            obj.obstacles{index} = tree;
        end
        
        
        function [c] = obstacleConstraint(obj,x)
           c=repmat(x(1),obj.number_of_obstacles,1);  % preallocate f (to overcome temporary limitation of TaylorVar)
           dc=zeros(obj.number_of_obstacles,length(x));
           for i=1:obj.number_of_obstacles
               c(i) = obj.obstacles{i}.spaceconstraint(x, obj.plane);
           end
        end
    end
end
