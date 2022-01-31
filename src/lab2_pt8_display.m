kine = Kinematics();
Q = readmatrix("lab2_pt8.csv");
times = Q(:, 1);
Q = Q(:, 2:4);
T = zeros(length(Q), 3);

for k = 1:length(Q)
    t = kine.fk3001(Q(k, :));
    T(k, :) = t(1:3, 4)';
end

plot(times, T(:, 1), times, T(:, 3));
title('X and Z Positions vs Time')
xlabel('Time (s)');
ylabel('Position (mm)');
legend('X', 'Z', 'Location', 'southeast');
figure();
plot(T(:, 1), T(:, 3))
title('X-Z 2d Trajectory Path')
xlabel('X Position (mm)');
ylabel('Z Position (mm)');
figure();
plot(T(:, 1), T(:, 2))
title('X-Y 2d Trajectory Path')
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
ylim([-10 10]);
figure();
plot(times, Q(:, 1), times, Q(:, 2), times, Q(:, 3));
title('Joint Angles vs Time')
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('X', 'Y', 'Z');
