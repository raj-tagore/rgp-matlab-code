rosshutdown; 
setenv('ROS_MASTER_URI','http://172.17.84.234:11311/'); setenv('ROS_IP','172.17.80.1');
rosinit("172.17.84.234");

global gripAct gripGoal trajAct trajGoal jointSub rgbSub pointsSub

[gripAct,gripGoal] = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory');
gripAct.FeedbackFcn = [];
[trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory');
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states");
rgbSub = rossubscriber("/camera/rgb/image_raw", "sensor_msgs/Image", "DataFormat", "struct");
pointsSub = rossubscriber("camera/depth/points", "sensor_msgs/PointCloud2", "DataFormat","struct");

global UR5e ik ikWeights initialIKGuess

UR5e = loadrobot('universalUR5e');
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));
tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e,"SolverAlgorithm","LevenbergMarquardt");
ikWeights = [0.25 0.25 0.25 0.1 0.1 0.1]; 

initialIKGuess = homeConfiguration(UR5e);
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal);
sendGoal(trajAct,trajGoal); 

load('RobocupDetector1');