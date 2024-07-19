function xyz_uv = get_xyz_uv_from_points()
    global pointsSub tfTree
    
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
end