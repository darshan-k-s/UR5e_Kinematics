% Project1PartE.m
% MTRN4230 Project 1 25T2
% Name: Darshan Komala Sreeramu
% Zid:  z5610741

pc = [1.4; 1; 0.8];    % target wrist-centre
r = norm(pc(1:2));    % horizontal reach
s = pc(3);    % vertical reach
a1 = 1;  
a2 = 1;   % link lengths

% Base yaw
q1 = atan2(pc(2), pc(1));     

% Two-link geometry
D = (r^2 + s^2 - a1^2 - a2^2)/(2*a1*a2);    % cosine of q3
fprintf('D = %.4f  (|D|<=1? %s)\n', D, string(abs(D)<=1))

q3_down = atan2(+sqrt(1-D^2), D);  % elbow down
q3_up = atan2(-sqrt(1-D^2), D);  % elbow up

beta = atan2(s, r);
gammaD = atan2(a2*sin(q3_down), a1 + a2*cos(q3_down));
gammaU = atan2(a2*sin(q3_up), a1 + a2*cos(q3_up));

q2Down = beta - gammaD;
q2Up = beta - gammaU;

% Forward check
xFK = @(q2,q3) cos(q1) + cos(q1+q2) + cos(q1+q2+q3);
zFK = @(q2,q3) sin(q2) + sin(q2+q3);

disp(' ')
fprintf('Elbow-DOWN solution:\n')
fprintf('  q1 = %7.3f°   q2 = %7.3f°   q3 = %7.3f°\n',...
        rad2deg([q1 q2Down q3_down]))
fprintf('  FK check  (x,z) = (%.3f, %.3f)\n',...
        xFK(q2Down,q3_down), zFK(q2Down,q3_down))

disp(' ')
fprintf('Elbow-UP   solution:\n')
fprintf('  q1 = %7.3f°   q2 = %7.3f°   q3 = %7.3f°\n', rad2deg([q1 q2Up q3_up]))
fprintf('  FK check  (x,z) = (%.3f, %.3f)\n', xFK(q2Up,q3_up), zFK(q2Up,q3_up))
