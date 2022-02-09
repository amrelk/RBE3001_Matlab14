targets = [100 0 195; 50 50 50; 50 100 100; 100 0 195];
n = 1;

q = [];
pos = [];
times = [];

robot.servo_jp([0 0 0]);
while ~robot.at_goal_js()
    pause(0.1);
end
tstart = tic;
while 1
    if robot.at_goal_js()
        if n > length(targets)
            break;
        else
            robot.interpolate_cp(targets(n, :), 0);
            n = n + 1;
        end
    else
        pos = [pos; robot.measured_cp(1)];
        q = [q; robot.measured_js()];
        times = [times; toc(tstart)];
    end
    pause(0.02);
end

figure();
plot(times, pos(:, 1));
hold on;
plot(times, pos(:, 2));
plot(times, pos(:, 3));
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
plot(times, q(:, 1));
hold on
plot(times, q(:, 2));
plot(times, q(:, 3));
hold off;
title('Joint Space Positions During Triangle');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Joint 1', 'Joint 2', 'Joint 3')
axis padded;
