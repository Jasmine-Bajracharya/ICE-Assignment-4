% Link lengths in metres for RTB (convert mm→m)
L1 = 99e-3;   % base to shoulder (d1)
L2 = 120e-3;  % shoulder to elbow (a2)
L3 = 195e-3;  % elbow to wrist (a3)
L6 = 60e-3;   % small tool offset (d6)

% Standard DH using Link('d',d, 'a',a, 'alpha',alpha)
L(1) =Link('d', L1, 'a', 0, 'alpha', pi/2);
L(2) =Link('d', 0, 'a', L2, 'alpha', 0);
L(3) =Link('d', 0, 'a', L3, 'alpha', 0);
L(4) =Link('d', 0, 'a', 0, 'alpha', pi/2);
L(5) =Link('d', 0, 'a', 0, 'alpha', pi/2);
L(6) =Link('d', 0, 'a', 0, 'alpha', 0 );


robot = SerialLink(L, 'name', 'RoboDigg6');

% Joint limits (conservative classroom limits; refine per hardware)
robot.qlim = deg2rad([ -170 170;  -90  90;  -120 120;  -180 180;  -120 120;  -180 180 ]);

% Example pose (degrees → radians)
q_deg = [ 30, 35, -10, 20, 10, -100 ];
q = deg2rad(q_deg);

% FK
T06 = robot.fkine(q);
disp('End-effector pose ^0T6:'); disp(T06);
disp('Position (x y z) in m:'); disp(transl(T06));

% Visualization
figure(1); clf; robot.plot(q, 'workspace', [-0.4 0.4 -0.4 0.4 0 0.5]);
trplot(T06, 'frame', '6', 'length', 0.05);
title('3.3 Robot configuration after applying transformation T');
view(135, 25); grid on;

hold on;
trplot(T06, 'frame', '6', 'color', 'r', 'length', 0.05);
title('3.4 End effector reference frame after applying transformation T');

% Animate a small joint-space move
q2 = q + deg2rad([10 -15 10 0 0 20]);
figure(2); clf;
while true
    % Forward move
    robot.plot([q; q2], 'fps', 20, 'trail', {'r', 'LineWidth', 2});
    pause(0.5); % short pause

    % Backward move (return to start)
    robot.plot([q2; q], 'fps', 20, 'trail', {'b', 'LineWidth', 2});
    pause(0.5);
end
