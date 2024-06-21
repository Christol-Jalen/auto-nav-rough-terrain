function cost = exampleHelperRolloverHeuristicTransition(motionSegment, costMap, weight)
%exampleHelperRolloverHeuristicTransition Penalize motion with transverse
%gradient for given set of plannerHybridAStar's motion primitives

% Copyright 2023 The MathWorks, Inc.

    states = nav.algs.hybridAStar.motionPrimitivesInterpolate(motionSegment, 1); 

    %Cost for each motion segments
    transitionCost = nan(size(states,3),1); 
    rolloverCost = nan(size(states,3),1);

    for i=1:size(states,3)

        xyState = squeeze(states(:,:,i));
        % Calculate distance-based transit cost
        
        xyPoints = xyState(:,1:2);
        thetas   = xyState(:,3);

        dXY = abs(xyPoints(2:end,:)-xyPoints(1:end-1,:));
        transitionCost(i) = sum(sqrt(sum(dXY.^2,2)));

        xyPoints = xyPoints(1:end-1,:);

        % XY flipped to calculate perpendicularity
        vGrad = -tan(thetas(2:end)-thetas(1:end-1)).*[getMapData(costMap,"yCost",xyPoints) getMapData(costMap,"xCost",xyPoints)];

        % Calculate a cost based on transverse slope
        rolloverCost(i) = sum(weight.*abs(dXY/norm(dXY).*vGrad),"all");
    end

    rolloverCost(isnan(rolloverCost)) = 0;

    % Combine transit/rollover costs
    cost = transitionCost + rolloverCost;
end