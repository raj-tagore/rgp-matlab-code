objectLocations = [
    -0.35 -0.007 0.23; % can
    -0.53 -0.05 0.08; % bottle
    -0.5 0.39 0.15; % can
    0.66 0.025 0.25; % can
    0.66 0.025 0.15; % can
    0.46 -0.07 0.24 % bottle
    ];
objectBins = ["green" "blue" "green" "green" "green" "blue"];
objectGripWidths = [0.228 0.228 0.228 0.228 0.228 0.517];
objectOrientations = ["y" "x" "x" "y" "y" "y"];

for i=1:length(objectBins)
    pickDrop(objectLocations(i,:),objectOrientations(i),objectGripWidths(i),objectBins(i));
end