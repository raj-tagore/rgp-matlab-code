imageSub = rossubscriber('/camera/depth/image_raw', 'sensor_msgs/Image');
figureHandle = figure('Name', 'ROS Image Viewer', 'NumberTitle', 'off');

imageSub.NewMessageFcn = @imageCallback;

while true
    pause(0.1); % Small pause to prevent overwhelming the CPU
end

function imageCallback(~, message)
    % Convert ROS Image message to MATLAB image
    img = readImage(message);
    
    % Display the image
    imshow(img);
    drawnow;
end