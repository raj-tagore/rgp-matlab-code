jointStateMsg = receive(jointSub,3)
initialIKGuess(1).JointPosition = jointStateMsg.Position(4);
initialIKGuess(2).JointPosition = jointStateMsg.Position(3);
initialIKGuess(3).JointPosition = jointStateMsg.Position(1);
initialIKGuess(4).JointPosition = jointStateMsg.Position(5);
initialIKGuess(5).JointPosition = jointStateMsg.Position(6);
initialIKGuess(6).JointPosition = jointStateMsg.Position(7);
show(UR5e,initialIKGuess)