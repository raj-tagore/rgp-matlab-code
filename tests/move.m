% for pose - x: right, y: straight, z: up. enter as [x y z rotz roty rotx]

function move(targetPose)
    global trajAct trajGoal

    % if targetOrientation == "x"
    %     ori = [0 1 0 0];
    % elseif targetOrientation == "y" && targetLocation(1) >= 0 
    %     ori = [0 0.71 -0.71 0];
    % elseif targetOrientation == "y" && targetLocation(1) < 0
    %     ori = [0 0.71 0.71 0];
    % else
    %     ori = [0.71 -0.71 0 0];
    % end
    
    tform = eye(4); 
    tform(1:3, 1:3) = eul2rotm(targetPose(4:6));
    tform(1:3,4) = targetPose(1:3)';
    %[configSoln, ~] =ik('tool0',tform,ikWeights,initialIKGuess);
    configSoln = customIK(tform);
    trajGoal = packTrajGoal(configSoln,trajGoal); 
    sendGoal(trajAct,trajGoal);
    pause(15)

end

