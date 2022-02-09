planner = Traj_Planner();

targets = [100 0 195; 50 50 50; 50 100 100; 100 0 195];
targetsq = [0 0 0; 45.0000 57.2488 40.4474; 63.4349 31.4658 21.9471; 0 0 0];

q = [];
jank = [];

robot.servo_jp([0 0 0]);
while ~robot.at_goal_js()
    pause(0.1);
end
tstart = tic;
for n = 2:length(targetsq)
    traj = [planner.cubic_traj(targetsq(n-1, 1), targetsq(n, 1), 0, 0, 0, 1)';
            planner.cubic_traj(targetsq(n-1, 2), targetsq(n, 2), 0, 0, 0, 1)';
            planner.cubic_traj(targetsq(n-1, 3), targetsq(n, 3), 0, 0, 0, 1)'];
    Q = robot.run_trajectory(traj, 1, 1);
    q = [q; Q];
    jank = [jank; repmat([n-2 0 0 0], length(Q), 1)];
end
q = q+jank;

p = kine.fk3001(q(1, 2:4));
pos = p(1:3, 4)';
vel = [0 0 0];
acc = [0 0 0];
for k = 2:length(q)
    p = kine.fk3001(q(k, 2:4));
    pos = [pos; p(1:3, 4)'];
    vel = [vel; (pos(k, :) - pos(k-1, :)) / (q(k, 1) - q(k-1, 1))];
    acc = [acc; (vel(k, :) - vel(k-1, :)) / (q(k, 1) - q(k-1, 1))];
end



figure();
plot(q(:, 1), pos(:, 1));
hold on;
plot(q(:, 1), pos(:, 2));
plot(q(:, 1), pos(:, 3));
hold off;
title('Task Space Positions During Triangle');
xlabel('Time (s)');
ylabel('Position (mm)');
legend('X', 'Y', 'Z')
axis padded;

figure();
plot3(pos(:, 1), pos(:, 2), pos(:, 3))
title('Task Space Positions During Triangle');
xlabel('X Pos (mm)');
ylabel('Y Pos (mm)');
zlabel('Z Pos (mm)');
axis padded;

figure();
plot(q(:, 1), q(:, 2));
hold on
plot(q(:, 1), q(:, 3));
plot(q(:, 1), q(:, 4));
hold off;
title('Joint Space Positions During Triangle');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Joint 1', 'Joint 2', 'Joint 3')
axis padded;

figure();
subplot(3, 1, 1);
plot(q(:, 1), pos(:, 1));
hold on;
plot(q(:, 1), pos(:, 2));
plot(q(:, 1), pos(:, 3));
hold off;
xlabel('Time (s)');
ylabel('Position (mm)');
legend('X', 'Y', 'Z');
title('Task Space Position During Trajectory');
subplot(3, 1, 2);
plot(q(:, 1), vel(:, 1));
hold on;
plot(q(:, 1), vel(:, 2));
plot(q(:, 1), vel(:, 3));
hold off;
title('Task Space Velocity During Trajectory');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
subplot(3, 1, 3);
plot(q(:, 1), acc(:, 1));
hold on;
plot(q(:, 1), acc(:, 2));
plot(q(:, 1), acc(:, 3));
hold off;
title('Task Space Acceleration During Trajectory');
xlabel('Time (s)');
ylabel('Acceleration (mm/s^2)');