% Project1PartD.m
% MTRN4230 Project 1 25T2
% Name: Darshan Komala Sreeramu
% Zid:  z5610741

syms theta1 theta2 theta3 real

A = cos(theta2) + cos(theta2 + theta3);
B = sin(theta2) + sin(theta2 + theta3);
C = sin(theta2 + theta3);
s1 = sin(theta1);
c1 = cos(theta1);

% Define the jacobian
Jv = [ -A*s1, -B*c1, -C*c1;
       A*c1, -B*s1, -C*s1;
       0,     A,     cos(theta2 + theta3) ];

disp(Jv);

% Compute and simplify the determinant
detJv = det(Jv);
detJvSimp = simplify(detJv);
disp(detJvSimp);

% Convert to a MATLAB function for plotting
theta1Val = pi/4;
detJvFunc = matlabFunction(subs(detJvSimp, theta1, theta1Val), 'Vars', [theta2, theta3]);

% Create a grid for theta2 and theta3
[Theta2, Theta3] = meshgrid(linspace(-pi, pi, 200), linspace(-pi, pi, 200));
DetVals = detJvFunc(Theta2, Theta3);

% Plot the determinant
figure;
surf(Theta2, Theta3, DetVals, 'EdgeColor', 'none');
hold on;

% Plot where determinant is zero
contour3(Theta2, Theta3, DetVals, [0 0], 'k', 'LineWidth', 2);
xlabel('\theta_2');
ylabel('\theta_3');
zlabel('det(J_v)');
title('Determinant of J_v as a function of \theta_2 and \theta_3');
colorbar;
view(3);
grid on;
hold off;
