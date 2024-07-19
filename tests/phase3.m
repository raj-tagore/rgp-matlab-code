%% move to take images
poseToTakePics = [0.45 0.45 0.45 1.8 -pi 0]; 
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

%% Depth Image Processing

xyz_uv = get_xyz_uv_from_depthimg();

for obj=1:size(labels)
    obj_points = xyz_uv(bboxes(obj,2):bboxes(obj,2)+bboxes(obj,4),bboxes(obj,1):bboxes(obj,1)+bboxes(obj,3),:);
    obj_points = reshape(obj_points,[],3);
    obj_pointcloud = pointCloud(obj_points);
    pcshow(obj_pointcloud);
    hold on;
    if any(obj_points(:,3)>0)
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