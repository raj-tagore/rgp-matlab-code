function configSoln = customIK(targetTform)

    global ik ikWeights initialIKGuess

    validSolution = false;
    maxAttempts = 100;
    attempt = 0;
    joint2Threshold = [-1.3 1.3];

    % jointPositionValues = [-1.5 0 0.3 0 0 0; 1.5 0 0.3 0 0 0];

    while ~validSolution && attempt < maxAttempts

        % Solve the inverse kinematics
        [configSoln, ~] = ik('tool0', targetTform, ikWeights, initialIKGuess);

        % Check if the solution meets the joint 2 constraint
        if configSoln(2).JointPosition >= joint2Threshold(1) && configSoln(2).JointPosition <= joint2Threshold(2)
            validSolution = true;
        else
            % Randomize the initial guess slightly and try again
            % for i=1:size(jointPositionValues)
            %     for j=1:6
            %         initialIKGuess(j).jointPosition = jointPositionValues(i,j);
            %     end
            % end
            for i=1:6
                initialIKGuess(i).JointPosition = rand();
            end
        end

        attempt = attempt + 1;
    end

    % If no valid solution is found, return empty
    if ~validSolution
        configSoln = [];
        warning('No valid IK solution found within the constraint.');
    end
end