function [defaultPlanner, heightAwarePlanner, gradientAwarePlanner, rolloverAwarePlanner] = ...
    exampleHelperCreatePlannerObject(plannerType, costMap, gWeight, hWeight)
%

%   Copyright 2023 The MathWorks, Inc.

% Create a binary occupancy grid 
map = getLayer(costMap,"terrainObstacles"); 

if strcmp(plannerType, "plannerHybridAStar")

    % Create a stateValidator object 
    validator = validatorOccupancyMap;
    
    % Assigning map to stateValidator object
    validator.Map = map; 
    
    validator.StateSpace.StateBounds = [map.XLocalLimits; map.YWorldLimits; [-pi pi]]; 
    
    % Assign stateValidator object to the plannerHybridAStar
    defaultPlanner = plannerHybridAStar(validator);
    
    %Create plannerHybridAStar object with custom transition cost function
    heightAwarePlanner = plannerHybridAStar(validator, ...
        "TransitionCostFcn",@(motionSegment)exampleHelperZHeuristicTransition(motionSegment, costMap,gWeight));
    
    gradientAwarePlanner = plannerHybridAStar(validator, ...
        "TransitionCostFcn", @(motionSegment)exampleHelperGradientHeuristicTransition(motionSegment, costMap,gWeight));
    
    rolloverAwarePlanner = plannerHybridAStar(validator, ...
        "TransitionCostFcn", @(motionSegment)exampleHelperRolloverHeuristicTransition(motionSegment, costMap,gWeight));


else

    defaultPlanner = plannerAStarGrid(getLayer(costMap,"terrainObstacles"));
    
    %Create plannerAStarGrid object with custom G and H cost functions
    heightAwarePlanner = plannerAStarGrid(getLayer(costMap,"terrainObstacles"), ...
        GCostFcn=@(s1,s2)exampleHelperZHeuristic(costMap,gWeight,s1,s2), ...
        HCostFcn=@(s1,s2)exampleHelperZHeuristic(costMap,hWeight,s1,s2));
    
    gradientAwarePlanner = plannerAStarGrid(getLayer(costMap,"terrainObstacles"), ...
        GCostFcn=@(s1,s2)exampleHelperGradientHeuristic(costMap,gWeight,s1,s2), ...
        HCostFcn=@(s1,s2)exampleHelperGradientHeuristic(costMap,hWeight,s1,s2));
    
    rolloverAwarePlanner = plannerAStarGrid(getLayer(costMap,"terrainObstacles"), ...
        GCostFcn=@(s1,s2)exampleHelperRolloverHeuristic(costMap,gWeight,s1,s2), ...
        HCostFcn=@(s1,s2)exampleHelperRolloverHeuristic(costMap,hWeight,s1,s2));
end

end
