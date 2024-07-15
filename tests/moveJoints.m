initialIKGuess(1).JointPosition = 0; % (-pi to pi).
initialIKGuess(2).JointPosition = 0; % don't exceed +1.5 and -1.5.
initialIKGuess(3).JointPosition = -1; 
initialIKGuess(4).JointPosition = 0;
initialIKGuess(5).JointPosition = 0; 
initialIKGuess(6).JointPosition = 0;

trajGoal = packTrajGoal(initialIKGuess,trajGoal);
sendGoal(trajAct,trajGoal); 