function cost = exampleHelperZHeuristic(costMap,weight,gridState1,gridState2)
%exampleHelperZHeuristic Ignore gradients, include height in cost

% Copyright 2021-2023 The MathWorks, Inc.

    assert(size(gridState1,1) == 1 && size(gridState2,1) == 1);
    
    % Get Z layer
    zMap = costMap.getLayer("Z");
    
    % Get XY states
    xyState = zMap.grid2world([gridState1(1:2);gridState2(1:2)]);

    % Get height difference
    Z = zMap.getMapData(xyState);

    % Calculate cost
    cost = sqrt(sum([xyState(2,:)-xyState(1,:) weight*(Z(2)-Z(1))].^2));
end
