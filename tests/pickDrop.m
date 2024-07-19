function pickDrop(poseAndGripwidth, bin)
    global gripAct gripGoal jointSub initialIKGuess

    poseAndGripwidth

    % reset initialIKGuess to current config
    jointMsg = receive(jointSub);
    currentJointStates = jointMsg.Position;
    for j = 1:numel(initialIKGuess)
        initialIKGuess(j).JointPosition = currentJointStates(j);
    end
    
    % go to pre-grasp
    targetPose = poseAndGripwidth(1:6);
    targetPose(3) = poseAndGripwidth(3)+0.2;
    move(targetPose);

    % go to grasp
    targetPose = poseAndGripwidth(1:6);
    move(targetPose);

    % grasp object
    gripGoal=packGripGoal(poseAndGripwidth(7),gripGoal);
    sendGoal(gripAct,gripGoal);
    pause(10)

    % go to pre-grasp again
    targetPose = poseAndGripwidth;
    targetPose(3) = poseAndGripwidth(3)+0.2;
    move(targetPose)
    
    % go to bin
    if bin == "blue"
        binPose = [0.5 -0.3 0.3 pi/2 pi 0];
    elseif bin == "green"
        binPose = [-0.5 -0.3 0.3 -pi/2 pi 0];
    end
    move(binPose);

    % drop
    gripGoal=packGripGoal(0,gripGoal);
    sendGoal(gripAct,gripGoal);
    pause(10)
end