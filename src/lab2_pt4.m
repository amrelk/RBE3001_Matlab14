Q = [];

robot.servo_jp([0 0 0]);
while ~robot.at_goal_js()
    pause(0.1);
end
for k = 1:20
    while ~robot.at_goal_js()
        pause(0.1);
    end
    if isequal(robot.goal_js(), [0 0 0])
        robot.interpolate_jp([45, 60, -40], 1000);
    else
        q = robot.measured_js(1, 0);
        Q = [Q; q(1, :)];
        robot.interpolate_jp([0, 0, 0], 1000);
    end
end

writematrix(Q, 'lab2_pt4.csv');