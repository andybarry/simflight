classdef FlatPlane
    properties
        xv
        yv
        color
    end
    methods
        function obj = FlatPlane(xs, ys, color)
            obj.xv = xs;
            obj.yv = ys;
            if (nargin>2)
                obj.color = color;
            else
                obj.color = 'black';
            end
        end
        %draws the plane, with an optional state vector argument
        function draw(obj, x)
          %{
            persistent hFig;
            if (isempty(hFig))
                hFig = sfigure(25);
                set(hFig,'DoubleBuffer', 'on');
            end
            if nargin<2
                x = zeros(12,1);
            end
            roll = x(4);
            xpts = [obj.xv+x(1) obj.xv(1)+x(1)]; 
            ypts = [cos(roll)*obj.yv+x(2) cos(roll)*obj.yv(1)+x(2)];
            plot(xpts,ypts, obj.color)
          %}
        end
        
    end
end
