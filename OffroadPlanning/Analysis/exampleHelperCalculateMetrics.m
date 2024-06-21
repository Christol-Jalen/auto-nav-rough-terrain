function statTable = exampleHelperCalculateMetrics(costMap,scenarioParams,paths,planningTime,pathNames,plannerType)
%exampleHelperCalculateMetrics Calculates metrics for one or more IJ or XY paths
% 
% Start by converting our XY states to surface-constrained SE3 states using
% exampleHelperXY2ConstrainedSE3. This helper function:
%   1) Calculates heading angle from XY motion
%   2) Projects the XY state onto the surface using a Z-height lookup
%   3) Calculates pitch/yaw by fixing yaw and align the robot's X and Z axis to the surface and surface-normal, respectively
%

% Copyright 2021-2023 The MathWorks, Inc.

    if nargin < 5
        pathNames = {};
    else
        pathNames = {'RowNames',cellstr(convertStringsToChars(pathNames))};
    end

    if ~iscell(paths)
        paths = {paths};
    end

    nPath = numel(paths);
    pathSE3 = cell(nPath,1);
    distance = nan(nPath,1);
    travelTime = nan(nPath,1);
    totalPower = nan(nPath,1);
    maxPitch = nan(nPath,1);
    maxRoll = nan(nPath,1);
    avgPitch = nan(nPath,1);
    avgRoll = nan(nPath,1);

    for i = 1:nPath
        % Convert path to XY
        if strcmp(plannerType, "plannerHybridAStar")
            pathXY = paths{i};
        else
            pathXY = grid2world(costMap,paths{i});
        end
        pathSE3{i} = exampleHelperXY2ConstrainedSE3(costMap,pathXY,plannerType);
        
        % Calculate distance
        distance(i) = sum(vecnorm(diff(pathSE3{i}(:,1:3)),2,2)); % meters
    
        % Calculate travel time
        travelTime(i) = (distance(i)/scenarioParams.Robot.Velocity)/60; % min
    
        % Calculate baseline energy consumption assuming constant rate of travel
        AH = scenarioParams.Robot.AmpHour;
        Vnom = scenarioParams.Robot.Vnom;
        energyBaseline = (AH*Vnom/3)*travelTime(i)*(3600/60); % J
        
        % Additional energy needed to climb hills (potential energy)
        deltaH = diff(pathSE3{i}(:,3));
        decline = deltaH < 0;
        deltaH(decline) = deltaH(decline)*scenarioParams.Robot.RegenerativeBrakeEfficiency;
        energyAdditional = scenarioParams.Robot.Mass*scenarioParams.Gravity*deltaH; % J
        
        % Total power consumption
        totalEnergy = energyBaseline + sum(energyAdditional); % J
        totalPower(i) = totalEnergy/(travelTime(i)*60); % W
        
        % Calculate max and average pitch and roll
        maxPitch(i) = max(abs(pathSE3{i}(:,5)))*180/pi;  % degrees
        avgPitch(i) = mean(abs(pathSE3{i}(:,5)))*180/pi; % degrees
        maxRoll(i)  = max(abs(pathSE3{i}(:,6)))*180/pi;  % degrees
        avgRoll(i)  = mean(abs(pathSE3{i}(:,6)))*180/pi; % degrees
    end
    statTable = table(distance,totalPower,travelTime,planningTime,maxPitch,avgPitch,maxRoll,avgRoll,...
        'VariableNames',{'Distance (m)','Power (W)','Travel Time (s)','Planning Time (s)','Max Pitch (deg)','Average Pitch (deg)','Max Roll (deg)','Average Roll (deg)'},pathNames{:});
end
