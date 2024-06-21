function exampleHelperVisualizeResults(ax,statTable,scenarioParams,costMap,paths,pathNames,animateResults, plannerType)
%exampleHelperVisualizeResults Display surface and animate vehicle following paths

% Copyright 2021-2023 The MathWorks, Inc.
    
    if nargin < 7
        animateResults = false;
    end
    
    % Display planning surface
    xLim = costMap.XWorldLimits;
    yLim = costMap.YWorldLimits;
    mapSize = costMap.GridSize;
    Z = flipud(getMapData(costMap,"Z"));
    X = linspace(xLim(1),xLim(2),mapSize(2));
    Y = linspace(yLim(1),yLim(2),mapSize(1));
    h(1) = surf(ax,X,Y,Z,"EdgeColor","none");
    
    hold on; 
    colormap("turbo");
    ax.ZLim = [min(Z,[],"all") max(Z,[],"all")+5];
    title("Planned Path"); xlabel("X [meters]"); ylabel("Y [meters]"); zlabel("Z [meters]"); zlim([0 13]);
    
    % Create and display terrain layer overlay
    occupiedMatrix = flipud(double(getMapData(costMap,"terrainObstacles"))); % Get occupancy values
    [occupiedMatrix, occColors] = formatObstacles(occupiedMatrix,Z,"occupied");
    h(2) = surf(X,Y,occupiedMatrix,occColors,"EdgeColor","none"); % Off-limit areas

    hasObstacleLayer = any(contains("obstacles",costMap.LayerNames));

    if hasObstacleLayer
        obstacleMatrix = flipud(double(getMapData(costMap,"obstacles"))); % Get occupancy values
        [obstacleMatrix, obsColors] = formatObstacles(obstacleMatrix,Z,"obstacles");
        h(3) = surf(X,Y,obstacleMatrix,obsColors,"EdgeColor","none","FaceAlpha",0.8); % Terrain obstacles
    end

    light(ax);
    ax.Clipping = 'off';
    arrayfun(@(x)set(x,'facelighting','none'),h);

    rotate(ax,[0 0 1],30);
    axis equal;
    nPath = numel(paths);
    cOrder = colororder(ax);

    if animateResults
        pathSE3 = cell(nPath,1);
        rbtTransform = cell(nPath,1);
        light(ax);
    end

    for j = 1:nPath

        if strcmp(plannerType, "plannerHybridAStar")
            xyPoints = paths{j};
        else
            %Convert path indices to XY states
            xyPoints = grid2world(costMap,paths{j});
        end

        % Plot path
        z = interp2(X,Y,Z,xyPoints(:,1),xyPoints(:,2));
        col = cOrder(mod(j-1,size(cOrder,1))+1,:);
        plot3(xyPoints(:,1),xyPoints(:,2),z,".","Color",col,"MarkerSize",12);
        
        if animateResults
            % Linearly interpolate XY path based on speed
            t = statTable.("Travel Time (s)")(j);
            d = [0:scenarioParams.UpdateRate:t t]'/t*statTable.("Distance (m)")(j);
            dXYZ = diff([xyPoints z]);
            dInterval = [0;cumsum(vecnorm(dXYZ,2,2))];
            dIdx = discretize(d,[-inf; dInterval(2:end-1); inf]);
            xyInterpolated = xyPoints(dIdx,1:2)+dXYZ(dIdx,1:2).*(d-dInterval(dIdx,:))./(dInterval(dIdx+1,:)-dInterval(dIdx,:));
            
            % Convert interpolated XY path to SE3 poses
            pathSE3{j} = exampleHelperXY2ConstrainedSE3(costMap,xyInterpolated,false);

            % Create robot handle
            plotTransforms([0 0 0],[1 0 0 0],"Parent",ax,"MeshFilePath","groundvehicle.stl","MeshColor",col);
            rbtGroup = ax.Children(1);

            % Update size to be ~1m
            rbtGroup.Children.Matrix = makehgtform('scale',2);
            
            % Raise robot
            rbtGroup.Children.Matrix(3,end) = 0.3;

            % Parent group to new transform for updateable pose
            rbtTransform{j} = hgtransform("Parent",ax);
            rbtGroup.Parent = rbtTransform{j};
        end
    end

    if hasObstacleLayer
        legendNames = ["Surface","Occupied Region","Obstacles",pathNames(:)'];
    else
        legendNames = ["Surface","Occupied Region",pathNames(:)'];
    end
    legend(ax,legendNames);
    view([-34.2093 17.5588]);

    if animateResults
        maxStep = max(cellfun(@(x)size(x,1),pathSE3));
        
        [~,fastRbt] = min(statTable.("Distance (m)"));
    
        % Visualize robots following each path
        tic;
        for i = 1:maxStep
            for j = 1:nPath
                if i <= size(pathSE3{j},1)
                    % Update robot pose
                    rbtTransform{j}.Matrix(1:3,:) = [eul2rotm(pathSE3{j}(i,4:end),'ZYX') pathSE3{j}(i,1:3)'];
                end
            end
            camtarget(rbtTransform{fastRbt}.Matrix(1:3,end)) % Set camera view to follow first vehicle closely
            if i == 1
                vCam = campos-camtarget;
                vCam = vCam/norm(vCam);
            end
            campos(rbtTransform{fastRbt}.Matrix(1:3,end)'+vCam*30)
            camva(25)
            drawnow;
            pause(max(0,scenarioParams.UpdateRate-toc));
            tic;
        end
    end

    hold off;
end

function [obstacleMatrix, colors] = formatObstacles(obstacleMatrix,Z,type)
%formatObstacles Create a 3D surface to overlay obstacle/terrain data atop surface
    
    % Start below surface
    baseOffset = 0.1;
    
    colors = zeros([size(Z) 3]);

    switch type
        case "occupied"
            % Set directly atop surface
            obsHeight = 1e-3;
        case "obstacles"
            % Raise from surface
            obsHeight = 2;

            % Set color to green
            colors(:,:,1) = ones(size(Z))*(76/255);
            colors(:,:,2) = ones(size(Z))*(115/255);
            colors(:,:,3) = ones(size(Z))*(25/255);
    end
    mFree = ~obstacleMatrix;
    obstacleMatrix(~mFree) = Z(~mFree)+obsHeight;
    obstacleMatrix(mFree) = Z(mFree)-baseOffset;
end
