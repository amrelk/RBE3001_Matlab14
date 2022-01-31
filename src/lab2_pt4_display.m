kine = Kinematics();
Q = readmatrix("lab2_pt4.csv");
T = zeros(length(Q), 3);

for k = 1:length(Q)
    t = kine.fk3001(Q(k, :));
    T(k, :) = t(1:3, 4)';
end

Ta = mean(T);
t = kine.fk3001([45 60 -40]);
Ti = t(1:3, 4)';

figure();
plot3(T(:, 1), T(:, 2), T(:, 3), 'o');
hold on;
plot3(Ta(1), Ta(2), Ta(3), '^');
plot3(Ti(1), Ti(2), Ti(3), '*');
hold off;
axis equal;
axis padded;
legend('Actual Positions', 'Mean Position', 'Ideal Position');
title('End Effector Position during 10 Movements');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

delta = Ti - Ta
RMS = [];
for k = 1:length(T)
    RMS = [RMS sqrt(immse(Ti, T(k, :)))];
end
RMS

