function cost = exampleHelperGradientHeuristic(costMap,weight,gridState1,gridState2)
%exampleHelperGradientHeuristic Penalizes motion along steep slope

% Copyright 2021-2023 The MathWorks, Inc.

    assert(size(gridState1,1) == 1 && size(gridState2,1) == 1);

    % Calculate distance-based transit cost
    xyState = costMap.grid2world([gridState1(1:2);gridState2(1:2)]);
    dXY = abs(xyState(2,:)-xyState(1,:));
    transitCost = sqrt(sum((dXY).^2));

    % hill cost
    if all(dXY == 1)
        % Diagonal motion, check diagonal cost map for visit cost
        hillCost = getMapData(costMap,"diagCost",xyState(1,:));
    else
        vGrad = [getMapData(costMap,"xCost",xyState(1,:)) getMapData(costMap,"yCost",xyState(1,:))];

        % Calculate a cost based on transverse slope
        hillCost = weight*abs(dXY/norm(dXY)*vGrad');
    end

    if isnan(hillCost)
        hillCost = 0;
    end

    cost = transitCost + hillCost*weight;
end
