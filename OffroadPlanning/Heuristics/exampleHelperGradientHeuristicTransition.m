function cost = exampleHelperGradientHeuristicTransition(motionSegment,costMap,weight, varargin)
%exampleHelperGradientHeuristicTransition Penalizes motion along steep slope
%for given set of plannerHybridAStar's motion primitives

% Copyright 2023 The MathWorks, Inc.

    states = nav.algs.hybridAStar.motionPrimitivesInterpolate(motionSegment, 1); 
    transitionCost = nan(size(states,3),1); 
    hillCost    = zeros(size(states,3),1);

    for i=1:size(states,3)

        xyState = squeeze(states(:,:,i));
        
        xyPoints = xyState(:,1:2);
        thetas   = xyState(:,3);
        
        % Calculate distance-based transit cost
        dXY = abs(xyPoints(2:end,:)-xyPoints(1:end-1,:));
        transitionCost(i) = sum(sqrt(sum(dXY.^2,2)));
        
        xyPoints = xyPoints(1:end-1,:);

        % Calculate a cost based on transverse slope
        vGrad = abs(tan(thetas(2:end)-thetas(1:end-1)).*[getMapData(costMap,"xCost",xyPoints) getMapData(costMap,"yCost",xyPoints)]);
        
        hillCost(i) = sum(weight.*abs(dXY./norm(dXY).*vGrad'),'all');
    end

    hillCost(isnan(hillCost)) = 0;

    cost = transitionCost + hillCost.*weight;
end
