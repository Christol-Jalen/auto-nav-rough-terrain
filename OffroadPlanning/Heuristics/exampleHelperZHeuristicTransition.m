function cost = exampleHelperZHeuristicTransition(motionSegment, costMap,weight) 
%exampleHelperZHeuristicTransition Ignore gradients, include height in cost
%for given set of plannerHybridAStar's motion primitives

% Copyright 2023 The MathWorks, Inc.

    states = nav.algs.hybridAStar.motionPrimitivesInterpolate(motionSegment, 1); 
    cost = nan(size(states,3),1); 

    % Get Z layer 
    zMap = costMap.getLayer("Z"); 

    for i=1:size(states,3) 
        XYPoints = squeeze(states(:,1:2,i)); 

        % Get height difference 
        Z = zMap.getMapData(XYPoints); 

        % Calculate cost 
        dXY = XYPoints(2:end,:)-XYPoints(1:end-1,:);
        dZ = weight*Z(2:end)-Z(1:end-1);
        cost(i) = sum(sqrt(sum([dXY dZ].^2,2)));

    end

end 