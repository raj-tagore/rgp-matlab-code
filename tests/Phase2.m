%%
initialIKGuess = homeConfiguration(UR5e);
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal);
sendGoal(trajAct,trajGoal); 
pause(15);

%%
desiredPose = [0 0.25 0.55 pi pi 0];
move(desiredPose);

%%
% desiredPos = [-1.8, 0.0, 1.0, -0.5, 0.76, 0.0, -0.5];
% for i=1:7
%     global initialIKGuess
%     initialIKGuess(i).JointPosition = desiredPos(i);
% end
% trajGoal = packTrajGoal(initialIKGuess,trajGoal);
% sendGoal(trajAct,trajGoal);

%%
rgbImageRaw = receive(rgbSub);
rgbImageMat = rosReadImage(rgbImageRaw);
[bboxes, scores, labels] = detect(trainedDetector, rgbImageMat);

high_score_indices = scores >= 0.5;
bboxes = bboxes(high_score_indices, :);
scores = scores(high_score_indices);
labels = labels(high_score_indices); 

[~, ~, numericLabels] = unique(labels);
combined_xbbox_labels = [bboxes(:,1), numericLabels];
pouch_removed_indices = combined_xbbox_labels(:,2)<3;
combined_xbbox_labels = combined_xbbox_labels(pouch_removed_indices,:);
sorted_xbbox_labels = sortrows(combined_xbbox_labels,1);
sorted_labels = sorted_xbbox_labels(:,2);

classNames = trainedDetector.ClassNames;
labelStrs = classNames(labels);
annotations = strcat(labelStrs, ': ', string(scores));
rgbImageMat = insertObjectAnnotation(rgbImageMat, 'rectangle', bboxes, annotations);
imshow(rgbImageMat);
hold on;

%%
% from left to right: 
% obj1 - [0.07, 0.23, 0.61] -> [-0.23 0.17 0.005] -> z orientation
% obj2 - [0.3, 0.15, 0.61] -> [-0.15 0.4 0.005] -> z orientation
% obj3 - [0.22 -0.02 0.54] -> [0.02 0.32 -0.07] -> y orientation
% obj4 - [0.26 -0.22 0.54] -> [0.22 0.36 -0.07] -> x orientation
% rbot - [-0.1 0 0.615]

poseAndGripWidths = [-0.23 0.17 0.08 -pi/2 pi 0 0.228
                -0.15 0.41 0.08 pi pi 0 0.228
                0.02 0.32 0.08 pi/2 pi 0 0.228
                0.22 0.36 0.08 pi pi 0 0.228];

obj_orientations = ['z','z','y','x'];
for i=1:4
    if obj_orientations(i)=="z"
        if sorted_labels(i)==1
            % if its a bottle in z, pick up via the top.
            % hence change height and gripwidth
            poseAndGripWidths(i,3) = 0.24;
            poseAndGripWidths(i,7) = 0.517; 
        else
            % if its a can in z, only change height
            poseAndGripWidths(i,3) = 0.15;
        end      
    end
end

obj_orientations = ["y" "x" "y" "x"];

obj_bins = strings(4);
obj_bins(sorted_labels==1) = "blue";
obj_bins(sorted_labels==2) = "green";

for i=1:length(poseAndGripWidths)
    pickDrop(poseAndGripWidths(i,:),obj_bins(i));
end
