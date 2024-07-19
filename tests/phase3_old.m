%% move to take images
poseToTakePics = [0.45 0.45 0.5 1.8 -pi 0]; 
move(poseToTakePics)
pause(10);

%% RGB Image processing

rgbImageRaw = receive(rgbSub);
rgbImageMat = rosReadImage(rgbImageRaw);
[bboxes, scores, labels] = detect(trainedDetector, rgbImageMat);

high_score_indices = scores >= 0.5;
bboxes = bboxes(high_score_indices, :);
scores = scores(high_score_indices);
labels = labels(high_score_indices); 

non_pouch_indices = labels ~= "pouch";
bboxes = bboxes(non_pouch_indices,:);
scores = scores(non_pouch_indices);
labels = labels(non_pouch_indices);

bboxes = round(bboxes);

classNames = trainedDetector.ClassNames;
labelStrs = classNames(labels);
annotations = strcat(labelStrs, ': ', string(scores));
rgbImageMat = insertObjectAnnotation(rgbImageMat, 'rectangle', bboxes, annotations);
imshow(rgbImageMat);

%% Depth Image processing

% get all points and convert them to a pointcloud object
points = receive(pointsSub);
xyz = rosReadXYZ(points);
ptCloud = pointCloud(xyz);

% form the transformation matrix for the camera pose
% also includes xyz coordinate correction ..
tfTree = rostf;
tform = getTransform(tfTree, 'base', 'camera_depth_link');
translation = [tform.Transform.Translation.X, ...
               tform.Transform.Translation.Y, ...
               tform.Transform.Translation.Z];
quaternion = [tform.Transform.Rotation.W, ...
              tform.Transform.Rotation.X, ...
              tform.Transform.Rotation.Y, ...
              tform.Transform.Rotation.Z];
rotMatrix = quat2rotm(quaternion);
tform1 = [rotMatrix, translation'; 0 0 0 1];
tform2 = [0 1 0 0; -1 0 0 0; 0 0 1 0; 0 0 0 1];
tform = tform2 * tform1;
rot = tform(1:3, 1:3);
trans = tform(1:3,4)';
geometricTform = rigid3d(rot, trans);

% Apply the final calculated transformation to the pointcloud
ptCloud_converted = pctransform(ptCloud, geometricTform);

pcshow(ptCloud_converted);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Point Cloud');

%%
% for each xyz calculate uv
K = [554.3827128226441 0.0 320.5; 0.0 554.3827128226441 240.5; 0.0 0.0 1.0];
points = ptCloud.Location;
world_points = ptCloud_converted.Location;

projectedPoints = (K * points')';
projectedPoints = projectedPoints ./ projectedPoints(:, 3);
projectedPoints = round(projectedPoints);

% form the xyz_uv matrix using xyz and uv data
xyz_uv = zeros(480,640,3);
for i=1:size(points)
    if any(isnan(projectedPoints(i)))
        continue;
    end
    xyz_uv(projectedPoints(i,2),projectedPoints(i,1),:) = world_points(i,:);
end

% for each object found, crop the xyz_uv and display it
for obj=1:size(labels)
    obj_points = xyz_uv(bboxes(obj,2):bboxes(obj,2)+bboxes(obj,4),bboxes(obj,1):bboxes(obj,1)+bboxes(obj,3),:);
    obj_points = reshape(obj_points,[],3);
    obj_pointcloud = pointCloud(obj_points);
    pcshow(obj_pointcloud);
    hold on;
    if any(obj_points(:,3)>0.03)
        angleRad = -1;
    else
        [K1, V1] = convhull(obj_points);
        hullPoints = obj_points(K1, :);
        covMatrix = cov(hullPoints);
        [eigVectors, eigValues] = eig(covMatrix);
        [sortedEigValues, sortIndices] = sort(diag(eigValues), 'descend');
        sortedEigVectors = eigVectors(:, sortIndices);
        principalAxis = sortedEigVectors(:,1);
        angleRad = -1*atan2(principalAxis(2), principalAxis(1));
    end
    
    disp(['Object:', labels(obj), 'with orientation:', num2str(angleRad), ' Radian ']);
end

