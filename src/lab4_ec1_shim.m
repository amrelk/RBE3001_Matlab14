planner = Vel_Planner();
kine = Kinematics();

targets = [100 0 195; 50 100 100; 100 50 50; 100 0 195];

c = [];
t = 0;
T = [];
cp = [0 0 0];
rpos = [0 0 0];
ts = 0.03;
for n = 2:length(targets)
    while ~isequal(abs(cp - targets(n, :)) <= 2, [1 1 1])
        js = rpos + (randn(1, 3)/1.5);
        cp = kine.fk3001(js);
        cp = cp(1:3, 4)';
        J = kine.jacob3001(js);
        vels = rad2deg(planner.inv_velkine(cp, targets(n, :), J)');
        rpos = (js + vels*ts);
        t = t+ts;
        T = [T; t];
        c = [c; cp];
    end
end

vel = [0 0 0];
acc = [0 0 0];
for k = 2:length(c)
    vel = [vel; (c(k, :) - c(k-1, :)) / (T(k) - T(k-1))];
    acc = [acc; (vel(k, :) - vel(k-1, :)) / (T(k) - T(k-1))];
end

figure();
plot3(c(:, 1), c(:, 2), c(:, 3));

figure();
subplot(3, 1, 1);
plot(T, c(:, 1));

axis padded;

subplot(3, 1, 2);
plot(T, c(:, 2));
axis padded;

subplot(3, 1, 3);
plot(T, c(:, 3));
axis padded;

figure();
subplot(3, 1, 1);
plot(T, c(:, 1));
hold on;
plot(T, c(:, 2));
plot(T, c(:, 3));
hold off;
xlabel('Time (s)');
ylabel('Position (mm)');
legend('X', 'Y', 'Z');
title('Task Space Position During Trajectory');
subplot(3, 1, 2);
plot(T, vel(:, 1));
hold on;
plot(T, vel(:, 2));
plot(T, vel(:, 3));
hold off;
title('Task Space Velocity During Trajectory');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
subplot(3, 1, 3);
plot(T, acc(:, 1));
hold on;
plot(T, acc(:, 2));
plot(T, acc(:, 3));
hold off;
title('Task Space Acceleration During Trajectory');
xlabel('Time (s)');
ylabel('Acceleration (mm/s^2)');