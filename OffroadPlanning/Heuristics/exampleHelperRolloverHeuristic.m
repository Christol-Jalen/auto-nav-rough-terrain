function cost = exampleHelperRolloverHeuristic(costMap,weight,gridState1,gridState2)
%exampleHelperRolloverHeuristic Penalize motion with transverse gradient

% Copyright 2021-2023 The MathWorks, Inc.

    assert(size(gridState1,1) == 1 && size(gridState2,1) == 1);

    % Calculate distance-based transit cost
    xyState = costMap.grid2world([gridState1(1:2);gridState2(1:2)]);
    dXY = abs(xyState(2,:)-xyState(1,:));
    transitCost = sqrt(sum((dXY).^2));

    % XY flipped to calculate perpendicularity
    vGrad = [getMapData(costMap,"yCost",xyState(1,:)) getMapData(costMap,"xCost",xyState(1,:))];

    % Calculate a cost based on transverse slope
    rolloverCost = weight*abs(dXY/norm(dXY)*vGrad');
    
    if isnan(rolloverCost)
        rolloverCost = 0;
    end

    % Combine transit/rollover costs
    cost = transitCost + rolloverCost;
end
