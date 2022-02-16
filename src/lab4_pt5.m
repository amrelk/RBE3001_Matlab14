planner = Traj_Planner();

targets = [100 0 195; 50 100 100; 0 0 294; 100 0 195];

q = [];
jank = [];

robot.servo_jp([0 0 0]);
while ~robot.at_goal_js()
    pause(0.1);
end
tstart = tic;
try
    for n = 2:length(targets)
        traj = [planner.quintic_traj(targets(n-1, 1), targets(n, 1), 0, 0, 0, 0, 0, 1)';
                planner.quintic_traj(targets(n-1, 2), targets(n, 2), 0, 0, 0, 0, 0, 1)';
                planner.quintic_traj(targets(n-1, 3), targets(n, 3), 0, 0, 0, 0, 0, 1)'];
        q = [q; robot.run_trajectory(traj, 1, 0, tstart)];
    end
catch e
    disp(e);
end

p = kine.fk3001(q(1, 2:4));
pos = p(1:3, 4)';
vel = [0 0 0];
acc = [0 0 0];
for k = 2:length(q)
    p = kine.fk3001(q(k, 2:4));
    pos = [pos; p(1:3, 4)'];
end

figure();
subplot(3, 1, 1);
plot(q(:, 1), q(:, 2));
hold on;
plot(q(:, 1), q(:, 3));
plot(q(:, 1), q(:, 4));
hold off;
title('Linear Velocities During Triangle');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
legend('X', 'Y', 'Z')
axis padded;

subplot(3, 1, 2);
plot(q(:, 1), q(:, 5));
hold on;
plot(q(:, 1), q(:, 6));
plot(q(:, 1), q(:, 7));
hold off;
title('Angular Velocities During Triangle');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('X', 'Y', 'Z')
axis padded;

subplot(3, 1, 3);
plot(q(:, 1), vecnorm([q(:, 2)'; q(:, 3)'; q(:, 4)']));
title('Linear Speed During Triangle');
xlabel('Time (s)');
ylabel('Speed (mm/s)');
axis padded;

figure();
subplot(3, 1, 3);
plot(q(:, 1), q(:, 8));
title('Jp Determinant Near Singularity');
ylabel('det(Jp)');
xlabel('Time (s)');
axis padded;

subplot(3, 1, 1:2);
plot3(pos(:, 1), pos(:, 2), pos(:, 3));
title('Position Trace Near Singularity');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis padded;