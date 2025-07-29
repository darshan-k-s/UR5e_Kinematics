% Project1PartC.m
% MTRN4230 Project 1 25T2
% Name: Darshan Komala Sreeramu
% Zid: z5610741

clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement your calculateMaxJointVelocity function below
% Based on the last digit of you zID, find the max joint velocity
% using the corresponding TCP velocity and joint position data.
% 'joint_pos\joint_pos_(Last digit of zID).mat'
% 'TCP_Vels\tcp_vels_(Last digit of zID).mat'

% Joint postions are given in rads in the following order:
% [base, shoulder, elbow, wrist 1, wrist 2, wrist 3]

% TCP Velocities are given in m/s and rads/s in the following order:
% [x, y, z, rx, ry, rz]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
% FOR WINDOWS, PATH HAS \
joint_poses = matfile('joint_pos\joint_pos_1.mat').jointPos;
TCP_vels = matfile('TCP_Vels\tcp_vels_1.mat').endVelocity;
%}

% Change the data file number based on your zID here!
joint_poses = matfile('joint_pos/joint_pos_1.mat').jointPos;
TCP_vels = matfile('TCP_Vels/tcp_vels_1.mat').endVelocity;

maxJointVel = calculateMaxJointVelocity(joint_poses, TCP_vels);
disp("Maximum Joint Angular Velocity Value:")
disp(maxJointVel)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% You must implement the following function
% Return the max joint angular velocity value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name: Darshan Komala Sreeramu
% Zid: z5610741
function maxJointVel = calculateMaxJointVelocity(joint_poses, TCP_vels)
    % Converting to N×6 from 6xN
    if size(TCP_vels, 1) == 6 && size(TCP_vels, 2) == size(joint_poses, 1) % Also verify if datasets have equal entries
        TCP_vels = TCP_vels';
    end

    % Define the unbroken arm
    Links(1) = Link('d', 0.1625, 'a', 0, 'alpha', pi/2);
    Links(2) = Link('d', 0, 'a', -0.425, 'alpha', 0);
    Links(3) = Link('d', 0, 'a', -0.3922, 'alpha', 0);
    Links(4) = Link('d', 0.1333, 'a', 0, 'alpha', pi/2);
    Links(5) = Link('d', 0.0997, 'a', 0, 'alpha', -pi/2);
    Links(6) = Link('d', 0.0996, 'a', 0, 'alpha', 0);
    arm = SerialLink(Links, 'name', 'UR5e_unbroken');

    numOfEntries = size(joint_poses, 1);
    jointVelo = zeros(numOfEntries, 6); % Make empty initial array

    % Loop through all data to find joint velocities
    for i = 1:numOfEntries
        q = joint_poses(i, :);  % (Dim: 1×6)
        Vtcp = TCP_vels(i, :)';  % already in base frame(Dim: 6x1 after transpose)

        % Base-frame Jacobian
        J = arm.jacob0(q);  % (Dim: 6×6)

        % joint velocity using the inverse(pinv instead of inv in case matrix isn't square)
        dq = pinv(J) * Vtcp;  % (Dim: 6×1)

        jointVelo(i, :) = dq';
    end

    % Search and store index of max mod value
    maxVal = max(abs(jointVelo), [], 'all');
    maxIndex = find(abs(jointVelo) == maxVal, 1, 'first');
    [sampleIndex, jointIndex] = ind2sub(size(jointVelo), maxIndex);
    
    % Extract the signed value at that point
    peakVel = jointVelo(maxIndex);

    % Print out max vel, which joint, and what index in dataset
    fprintf("Max joint velocity = %+0.4f rad/s on joint %d at sample %d.\n", peakVel, jointIndex, sampleIndex);

    maxJointVel = peakVel;
end
