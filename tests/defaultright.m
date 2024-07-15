initialIKGuess(1).JointPosition=-0.69;
initialIKGuess(2).JointPosition=-0.3;
initialIKGuess(3).JointPosition=0.99;
initialIKGuess(4).JointPosition=-0.69;
trajGoal = packTrajGoal(initialIKGuess,trajGoal);
sendGoal(trajAct,trajGoal);
pause(10);