function pickDrop(objectLocation, objectOrientation, objectGripWidth, bin)
    global gripAct gripGoal jointSub initialIKGuess

    objectLocation, objectOrientation

    % reset initialIKGuess to current config
    jointMsg = receive(jointSub);
    currentJointStates = jointMsg.Position;
    for j = 1:numel(initialIKGuess)
        initialIKGuess(j).JointPosition = currentJointStates(j);
    end
    
    % go to pre-grasp
    targetPos = [objectLocation(1) objectLocation(2) objectLocation(3)+0.2];
    move(targetPos, objectOrientation);

    % go to grasp
    targetPos = objectLocation;
    move(targetPos, objectOrientation);

    % grasp object
    gripGoal=packGripGoal(objectGripWidth,gripGoal);
    sendGoal(gripAct,gripGoal);
    pause(10)

    % go to pre-grasp again
    targetPos = [objectLocation(1) objectLocation(2) objectLocation(3)+0.2];
    move(targetPos, objectOrientation)
    
    % go to bin
    if bin == "blue"
        binPos = [0.5 -0.3 0.3];
        binOri = [0 0.71 -0.71 0];
    elseif bin == "green"
        binPos = [-0.5 -0.3 0.3];
        binOri = [0 0.71 0.71 0];
    end
    move(binPos, "y");

    % drop
    gripGoal=packGripGoal(0,gripGoal);
    sendGoal(gripAct,gripGoal);
    pause(10)
end