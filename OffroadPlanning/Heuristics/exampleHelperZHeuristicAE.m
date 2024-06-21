function cost = exampleHelperZHeuristicAE(rsSegment, costMap,weight) 
%exampleHelperZHeuristicAE Ignore gradients, include height in cost
%for given Reeds-Shepp path segment

% Copyright 2023 The MathWorks, Inc.

    states = rsSegment.interpolate(0:1:rsSegment.Length); 

    % Get Z layer 
    zMap = costMap.getLayer("Z"); 

    XYPoints = states(:,1:2); 

    % Get height difference 
    Z = zMap.getMapData(XYPoints); 

    % Calculate cost 
    dXY = XYPoints(2:end,:)-XYPoints(1:end-1,:);
    dZ = weight*Z(2:end)-Z(1:end-1);
    cost = sum(sqrt(sum([dXY dZ].^2,2)));

end
