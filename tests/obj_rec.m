rgbImgRaw = receive(rgbSub);
rgbImgMatrix = rosReadImage(rgbImgRaw);
[bboxes, scores, labels] = detect(detector.detectorMult, rgbImgMatrix);

classNames = detector.detectorMult.ClassNames;
labelStrs = classNames(labels);
annotations = strcat(labelStrs, ': ', string(scores));
rgbImgMatrix = insertObjectAnnotation(rgbImgMatrix, 'rectangle', bboxes, annotations);
[m, n] = size(bboxes);
centers = zeros(m, 2);
mat=zeros(m);
for i = 1:m
    centers(i, 1) = bboxes(i, 1) + (bboxes(i, 3) / 2);
    centers(i, 2) = bboxes(i, 2) + (bboxes(i, 4) / 2);
end

imshow(rgbImgMatrix);
hold on;

