rgb_image_sub = rossubscriber("/camera/rgb/image_raw", "sensor_msgs/Image", "DataFormat", "struct");
rgb_image_raw = receive(rgb_image_sub);
rgb_image_matlab = rosReadImage(rgb_image_raw);
imshow(rgb_image_matlab);

depth_image_sub = rossubscriber("/camera/depth/image_raw", "sensor_msgs/Image", "DataFormat", "struct");
depth_image_raw = receive(depth_image_sub);
depth_image_matlab = rosReadImage(depth_image_raw);
imshow(depth_image_matlab);

