function xyz_uv = get_xyz_uv_from_depthimg()
    global depthSub tfTree

    depthImageRaw = receive(depthSub);
    depthImageMat = rosReadImage(depthImageRaw);
    K = [554.3827128226441 0.0 320.5; 0.0 554.3827128226441 240.5; 0.0 0.0 1.0];
    xyz_uv = zeros(480,640,3);
    
    for u=1:size(depthImageMat,2)
        for v=1:size(depthImageMat,1)
            pixel = [u,v,1];
            xy1 = K\pixel';
            xyz = xy1*(depthImageMat(v,u));
            xyz_uv(v,u,:)=xyz;
        end
    end

    % Above xyz_uv needs to be transformed
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
    geometricTform = rigidtform3d(rot, trans);

    xyz_uv = reshape(xyz_uv,[],3);
    xyz_uv = transformPointsForward(geometricTform,xyz_uv);
    xyz_uv = reshape(xyz_uv,480,640,3);

