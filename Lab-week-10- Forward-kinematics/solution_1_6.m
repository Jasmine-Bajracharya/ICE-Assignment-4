% DH parameters
d = [99 0 0 0 0 60];
a = [0 120 195 0 0 0];
alpha = [pi/2 0 0 pi/2 -pi/2 0];

pose_deg = [30 35 -10 20 10 -100];
theta = deg2rad(pose_deg);   % converting degrees to radians

% To build the matrix
dh = @(th, d, a, al)[
    cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
    sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
    0        sin(al)          cos(al)          d;
    0        0                0                1];

% transformation matrices
A1 = dh(theta(1), d(1), a(1), alpha(1));
A2 = dh(theta(2), d(2), a(2), alpha(2));
A3 = dh(theta(3), d(3), a(3), alpha(3));
A4 = dh(theta(4), d(4), a(4), alpha(4));
A5 = dh(theta(5), d(5), a(5), alpha(5));
A6 = dh(theta(6), d(6), a(6), alpha(6));

% Final transformation T0_6
T06 = A1*A2*A3*A4*A5*A6;

% Extracting position of end effector
pos = T06(1:3,4);

disp('Transformation matrix T0_6 = ')
disp(T06)
disp('End-effector position [x y z] (mm) = ')
disp(pos.')
