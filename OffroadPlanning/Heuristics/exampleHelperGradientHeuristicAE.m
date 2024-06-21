function cost = exampleHelperGradientHeuristicAE(rsSegment,costMap,weight, varargin)
%exampleHelperGradientHeuristicAE Penalizes motion along steep slope
%for given Reeds-Shepp path segment

% Copyright 2023 The MathWorks, Inc.

    states = rsSegment.interpolate(0:1:rsSegment.Length);
    
    xyPoints = states(:,1:2);
    thetas   = states(:,3);
    
    % Calculate distance-based transit cost
    dXY = abs(xyPoints(2:end,:)-xyPoints(1:end-1,:));
    transitionCost = sum(sqrt(sum(dXY.^2,2)));
    
    xyPoints = xyPoints(1:end-1,:);

    % Calculate a cost based on transverse slope
    vGrad = abs(tan(thetas(2:end)-thetas(1:end-1)).*[getMapData(costMap,"xCost",xyPoints) getMapData(costMap,"yCost",xyPoints)]);
    
    hillCost = sum(weight.*abs(dXY./norm(dXY).*vGrad'),'all');

    if isnan(hillCost)
        hillCost = 0;
    end

    cost = transitionCost + hillCost*weight;
end
