function poseSE3 = exampleHelperXY2ConstrainedSE3(costMap,pathXY,plannerType)
%exampleHelperXY2ConstrainedSE3 Converts the xy path to a sequence of [X Y Z eZ eY eX] states.
%
%   Z(i)   : vertical projection of [X(i) Y(i)] onto the surface
%   eZ(i)  : heading angle (yaw) along [X(i+1) Y(i+1)]-[X(i) Y(i)]
%   eY/eX  : pitch/roll formed by aligning robot Z-axis to surface normal
%

% Copyright 2021-2023 The MathWorks, Inc.
    
    if strcmp(plannerType,"plannerHybridAStar")
        yaw = pathXY(:,3);
    else
        % Calculate heading angle
        dXY = diff(pathXY);
        yaw = atan2(dXY(:,2),dXY(:,1));
        yaw = [yaw(1); yaw];
    end

    % Get interpolated path Z
    gSize = costMap.GridSize;
    IJ = repmat((1:max(gSize))',1,2);
    XY = grid2world(costMap,IJ);
    ZZ = getMapData(costMap,'Z');
    X = XY(1:gSize(2),1); Y = XY(end-gSize(1)+1:end,2);
    zPath = interp2(X,Y,ZZ,pathXY(:,1),pathXY(:,2));

    % Get interpolated gradient
    dzdx = getMapData(costMap,'dzdx');
    dzdy = getMapData(costMap,'dzdy');
    gx = interp2(X,Y,dzdx,pathXY(:,1),pathXY(:,2));
    gy = interp2(X,Y,dzdy,pathXY(:,1),pathXY(:,2));

    % Calculate constrained pitch/roll
    [yaw,pitch,roll] = getPitchRoll(gx,gy,yaw);

    % Create SE3 pose
    poseSE3 = [pathXY(:,1:2) zPath yaw pitch roll]; % yaw, pitch, roll
end

function [yaw,pitch,roll] = getPitchRoll(dx,dy,theta)
%getPitchRoll Align body-Z to surface normal with constrained-yaw
    maxN = max([numel(dx),numel(dy),numel(theta)]);

    zs = zeros(maxN,1);
    os = ones(maxN,1);
    
    dx = repmat(dx(:),maxN/numel(dx),1);
    dy = repmat(dy(:),maxN/numel(dy),1);
    theta = repmat(theta(:),maxN/numel(theta),1);
    v1 = [cos(theta) sin(theta) zs];
    
    % Create surface normal
    DX = [os zs dx];
    DY = [zs os dy];
    v2 = cross(DX./vecnorm(DX,2,2), DY./vecnorm(DY,2,2));
    v2 = v2./vecnorm(v2,2,2);
    
    % Convert orientation to normalized XY heading-vector
    yaw = theta;
    dZ = [dx dy zs];
    delZ = sum(v1.*dZ,2);
    pitch = -atan(delZ);
    
    % Rotate zUp=[0 0 1] using pitch angle, then calculate angle between zUpNew
    % and surfaceNormal
    Rtmp = eul2rotm([theta pitch zs]);
    yTmp = reshape(Rtmp(:,2,:),3,[]);

    % Calculate roll angle
    roll = acos(sum(v2.*yTmp',2))-pi/2;
end
