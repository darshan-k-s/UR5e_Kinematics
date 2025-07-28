% Project1PartA.m
% MTRN4230 Project 1 25T2
% Name: Darshan Komala Sreeramu
% Zid: z5610741

clear; clc;
startup_rvc;

%host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
rtde = rtde(host, port);

disp("Your elbow joint is stuck at 90 degrees!")

disp("Enter the pickup position")
pickupJointConfiguration = readConfiguration();

disp("Set robot to remote control mode then click enter")
input('');

clc;
pickupPose = convertJointToPose(pickupJointConfiguration);


jointAngles = [pickupJointConfiguration(1:2); 90; pickupJointConfiguration(3:end)];
rtde.movej(deg2rad(jointAngles)', 'joint');
rtde.movej(deg2rad(jointAngles)', 'joint');

clc;
% get real robot pose
realPose = rtde.actualPosePositions();


% RTDE says it's taking in [x,y,z,r,p,y] but 
% its actually taking in [x,y,z,(axis-angle rotation vector)]
% The below four lines converts students rpy pose, into one with a rotation
% vector, this requires the computer vision toolbox
Tp = rpy2tr(pickupPose(4:6));
pickupPose_converted = [pickupPose(1:3), rotmat2vec3d(Tp(1:3, 1:3))];

% Check student and real pose
checkSolution(realPose, pickupPose_converted)


% Function to convert user input to array
function configuration = readConfiguration()
    configuration = [];

    in = input('Enter joint configuration exactly in the form "j1,j2,j4,j5,j6": ', 's');
    joints = split(in, ",");

    for joint = joints
        configuration = [configuration, str2double(joint)];
    end
end

% Function for checking if solutions are correct. Do not modify!!!
% Look out for cases were -2.2214   -2.2214 == 2.2214    2.2214 for R and P
% values.
function correct = checkSolution(realPose, calucatedPose_converted)
    if realPose == calucatedPose_converted
        disp("Correct solution!")
    else
        i = 1;
        while i <= size(realPose,2)
            if abs(realPose(i) - calucatedPose_converted(i)) > 0.005
                errorString = "Mismatch between calculated an real solution at Pose value #" + i;
                realString = "The real value is " + realPose(i);
                yourString = "Your value is " + calucatedPose_converted(i);
                disp(errorString)
                disp(realString)
                disp(yourString)
            else
                disp("Correct solution, within tolerance!")
            end
            i = i+1;
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% You must implement the following function
% Output pose should be in the form [x, y, z, r, p, y] with rpy in radians.
% Remember to view pose using 'base' view in the simulator.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function outputPose = convertJointToPose(jointConfiguration)

    % jointConfiguration:- [j1, j2, j4, j5, j6] (Degrees)
    angles = deg2rad(jointConfiguration(:));
    
    %
    % Defining DH params
    %
    % Joint angle for 6 joints
    theta = [
        angles(1);
        angles(2);
        pi/2; % set at +90 deg
        angles(3);
        angles(4);
        angles(5)
        ];    
    % Link offset
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];  
    % Link length
    a = [0, -0.425, -0.3922, 0, 0, 0];
    % Link twist
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

    
    % Make homogeneous transform matrix from DH and chain up till joint 6
    T = eye(4); % Identity mat
    for i = 1:6
        th = theta(i);
        H = [ 
              cos(th), -sin(th)*cos(alpha(i)), sin(th)*sin(alpha(i)), a(i)*cos(th);
              sin(th), cos(th)*cos(alpha(i)), -cos(th)*sin(alpha(i)), a(i)*sin(th);
              0, sin(alpha(i)), cos(alpha(i)), d(i);
              0, 0, 0, 1
             ];

        T = T * H;
    end
    
    % Get position
    pos = T(1:3,4)';
    
    % Get rotation-matrix and convert to rpy
    % from (ZYX)
    R = T(1:3,1:3);
    r = atan2(R(3,2), R(3,3));
    p = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    y = atan2(R(2,1), R(1,1));
    
    % Return pose
    outputPose = [pos, r, p, y];
end

