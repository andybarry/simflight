function h=plotFunnel3(obj,options)
% Plots the one-level set of V in 3d
%
% @param options options structure
%
% @option plotdims coordinates along which to plot given as a 1x3 array
%   @dull [1, 2, 3]
% @option x0 dull coordinates (e.g. to be used in the slice)
% @option inclusion = { 'slice' | 'projection' }
%    'slice' -- include the points in the plane of the given
%        dimensions for which V(x) < 1.
%    'projection' -- plot the projection of {x | V(x) < 1} into the
%        given plane.
% @option tol tolerance for computing the level set
%
% @retval h column vector of handles for any graphics objects created

% todo: support wrapping coordinates

if (nargin<2) options=struct(); end
if ~isfield(options,'plotdims') options.plotdims = [1;2;3]; end
if ~isfield(options,'x0') options.x0 = zeros(obj.getFrame.dim,1); end
if (~isfield(options,'inclusion')) options.inclusion = 'slice'; end
if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
if (~isfield(options,'tol')) options.tol = .01; end
if (~isfield(options,'inflate_radius')) options.inflate_radius = 0.0; end

if length(options.plotdims) ~= 3
    error('Handles 3d plots only.')
end

% Setup colormaps
cmap = colormap('bone');
cmapdark = cmap(1:end-10,:);
cmaplight = cmap(10:end,:);

hold on;
view(3);
% view(0,90);

if isTI(obj)
    % TODO: Here we split between projection and slice.
    if strcmp(options.inclusion,'slice')
        x=getLevelSet(obj,0,options);
    elseif strcmp(options.inclusion,'projection')
        x=getProjection(obj,0,options.x0,options.plotdims,options);
    else
        error(['Unknown inclusion method: ' options.inclusion]);
    end
    
    if options.inflate_radius ~= 0.0
    % Add in inflate_radius
    x_origin_centered = x - repmat(mean(x,2),1,size(x,2));
    norms = sqrt(sum(x_origin_centered(1,:).^2 + x_origin_centered(2,:).^2 + x_origin_centered(3,:).^2,2));
    x_origin_centered = x_origin_centered + options.inflate_radius*(x_origin_centered./[norms;norms;norms]);
    x = x_origin_centered + repmat(mean(x,2),1,size(x,2));
    end
    
    % Compute convex hull of points
    K = convhulln(x');
    
    
    % Plot with trisurf
    h = trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.3,'EdgeAlpha',0.9);
    colormap(cmapdark);
    
    % Label axes
    xlabel(obj.getFrame.coordinates{options.plotdims(1)});
    ylabel(obj.getFrame.coordinates{options.plotdims(2)});
    zlabel(obj.getFrame.coordinates{options.plotdims(3)});
else
    if ~isfield(options,'ts') error('you must specify the time samples for this system using options.ts'); end
    ts = options.ts;
    
    if isnumeric(options.x0) options.x0 = ConstantTrajectory(options.x0); end
    x0 = options.x0;
    
    h=[];
    for i=length(ts)-1:-1:1
        if strcmp(options.inclusion,'slice')
            options.x0 = x0.eval(ts(i));
            xfun0 = getLevelSet(obj,ts(i),options);
            options.x0 = x0.eval(ts(i+1)-eps);
            xfun1 = getLevelSet(obj,ts(i+1)-eps,options);
            
            if options.inflate_radius ~= 0.0
            % Add in inflate_radius
            x_origin_centered = xfun0 - repmat(mean(x,2),1,size(x,2));
            norms = sqrt(sum(x_origin_centered(1,:).^2 + x_origin_centered(2,:).^2 + x_origin_centered(3,:).^2,2));
            x_origin_centered = x_origin_centered + options.inflate_radius*(x_origin_centered./[norms;norms;norms]);
            xfun0 = x_origin_centered + repmat(mean(x,2),1,size(x,2));
            end
            
            if options.inflate_radius ~= 0.0
            % Add in inflate_radius
            x_origin_centered = xfun1 - repmat(mean(x,2),1,size(x,2));
            norms = sqrt(sum(x_origin_centered(1,:).^2 + x_origin_centered(2,:).^2 + x_origin_centered(3,:).^2,2));
            x_origin_centered = x_origin_centered + options.inflate_radius*(x_origin_centered./[norms;norms;norms]);
            xfun1 = x_origin_centered + repmat(mean(x,2),1,size(x,2));
            end
            
            % opts = options;
            % opts.plotdims = options.plotdims(2:3);
            % xring = getLevelSet(obj,ts(i+1),opts);
        elseif strcmp(options.inclusion,'projection')
            xfun0 = getProjection(obj,ts(i),x0.eval(ts(i)),options.plotdims,options);
            xfun1 = getProjection(obj,ts(i+1)-eps,x0.eval(ts(i+1)-eps),options.plotdims,options);
            % xring = getProjection(obj,ts(i+1),x0.eval(ts(i+1)),options.plotdims(2:3),options);
            
            % Add in inflate_radius
            x_origin_centered = xfun0 - repmat(mean(xfun0,2),1,size(xfun0,2));
            norms = sqrt(sum(x_origin_centered(1,:).^2 + x_origin_centered(2,:).^2 + x_origin_centered(3,:).^2,1));
            x_origin_centered = x_origin_centered + options.inflate_radius*(x_origin_centered./[norms;norms;norms]);
            xfun0 = x_origin_centered + repmat(mean(xfun0,2),1,size(xfun0,2));
            
            
            % Add in inflate_radius
            x_origin_centered = xfun1 - repmat(mean(xfun1,2),1,size(xfun1,2));
            norms = sqrt(sum(x_origin_centered(1,:).^2 + x_origin_centered(2,:).^2 + x_origin_centered(3,:).^2,1));
            x_origin_centered = x_origin_centered + options.inflate_radius*(x_origin_centered./[norms;norms;norms]);
            xfun1 = x_origin_centered + repmat(mean(xfun1,2),1,size(xfun1,2));
            
            
        else
            error(['Unknown inclusion method: ' options.inclusion]);
        end
        
        if (i == (length(ts)-1))
            x = xfun1;
            K = convhulln(x');
            % hi = trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.6,'EdgeAlpha',0); % 0.6
            hi = trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.6,'EdgeAlpha',0,'FaceLighting','gouraud','AmbientStrength',0.9); % 0.6
            h = [h;hi];
            colormap(cmap);
            material dull
            % set(hi,'colormap',cmapdark');
        end
        
        x = [xfun0,xfun1(:,end:-1:1)];
        K = convhulln(x');
        %     h = [h;trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.2,'EdgeAlpha',0,'FaceLighting','flat')];
        h = [h;trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.2,'EdgeAlpha',0,'FaceLighting','gouraud','AmbientStrength',0.9)];
        colormap(cmap);
        material dull
        
        % x01 = x0.eval(ts(i+1)); x01 = x01(options.plotdims(1));
        % plot3(repmat(x01,1,length(xring)), xring(1,:), xring(2,:),'k','LineWidth',2);
        
        x = xfun0;
        K = convhulln(x');
        %     h = [h;trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.3,'EdgeAlpha',0,'FaceLighting','flat')]; % 0.3
        h = [h;trisurf(K,x(1,:),x(2,:),x(3,:),'FaceColor','interp','EdgeColor','interp','FaceAlpha',0.1,'EdgeAlpha',0,'FaceLighting','gouraud','AmbientStrength',0.9)];
        % colormap(cmapdark);
        colormap(cmap);
        material dull
        
    end
    
    % Flip colormap since we flip axes (usually)
    cmap = colormap;
    colormap(flipud(cmap));
    
    material dull;
    
    % Add some lighting
    % keyboard;
    % light('Position',[-1;-3;-5]); % ,'Style','local')
    % light('Position',[-1;3;-5]);
    % camlight
    
    
    
    
    % Plot rings at the beginning and end
    
    %   if strcmp(options.inclusion,'slice')
    %       opts = options;
    %       opts.x0 = x0.eval(ts(1));
    %       opts.plotdims = options.plotdims(2:3);
    %       xring = getLevelSet(obj,ts(1),opts);
    %   else
    %       xring = getProjection(obj,ts(1),x0.eval(ts(1)),options.plotdims(2:3),options);
    %   end
    %
    %   x01 = x0.eval(ts(1)); x01 = x01(options.plotdims(1));
    %   plot3(repmat(x01,1,length(xring)), xring(1,:), xring(2,:),'k','LineWidth',2);
    
    
    %   if strcmp(options.inclusion,'slice')
    %       opts = options;
    %       opts.x0 = x0.eval(ts(end));
    %       opts.plotdims = options.plotdims(2:3);
    %       xring = getLevelSet(obj,ts(end),opts);
    %   else
    %       xring = getProjection(obj,ts(end),x0.eval(ts(end)),options.plotdims(2:3),options);
    %   end
    %
    %   x01 = x0.eval(ts(end)); x01 = x01(options.plotdims(1));
    %   plot3(repmat(x01,1,length(xring)), xring(1,:), xring(2,:),'k','LineWidth',2);
end

