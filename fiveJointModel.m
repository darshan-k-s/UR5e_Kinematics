% Project1PartB.m
% MTRN4230 Project 1 25T2
% Name: Darshan Komala Sreeramu
% Zid:  z5610741

r = -1 * sqrt(0.425^2 + 0.3922^2);  % –0.5783

% Define the new 5 joint arm
L(1) = Link('d', 0.1625, 'a', 0, 'alpha', pi/2);
L(2) = Link('d', 0, 'a', r, 'alpha', 0);
L(3) = Link('d', 0.1333, 'a', 0, 'alpha', pi/2);
L(4) = Link('d', 0.0997, 'a', 0, 'alpha', -1 * pi/2);
L(5) = Link('d', 0.0996, 'a', 0, 'alpha',  0   ); 

% Create 5-joint serial link
newArm = SerialLink([L(1) L(2) L(3) L(4) L(5)], 'name', 'Broken UR5e (5DOF)');

% Home joint configuration: θs are added here
qHome = deg2rad([0, 42.702 - 75, 47.298 - 105, 90, 0]);
% [0, -75, -105, 90, 0] is the given home config

% Forward kinematics
T = newArm.fkine(qHome);
disp('Final transformation matrix 0T5:');
disp(T);

pos = transl(T);
orientation = tr2rpy(T, 'zyx');
% Range conversion from [-pi, pi] to [0, 2pi]
orientation(3) = mod(orientation(3), 2*pi); 

fprintf('Position(m) = [%.4f  %.4f  %.4f]\n', pos);
fprintf('RPY(rad)    = [%.3f  %.3f  %.3f]\n', orientation);

% Plot the arm in RVC
figure;
newArm.plot(qHome);
title('Broken UR5e (Elbow Collapsed)');
