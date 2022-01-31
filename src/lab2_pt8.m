Q = [];
pts = [0 0 0; 0 42 -6; 0 38 50; 0 0 0];
n = 0;

robot.servo_jp([0 0 0]);
while ~robot.at_goal_js()
    pause(0.1);
end
tstart = tic;
while 1
    if robot.at_goal_js()
        if n < length(pts)
            n = n+1;
        else
            break;
        end
        robot.interpolate_jp(pts(n, :), 1000);
    end
    q = robot.measured_js(1, 0);
    Q = [Q; toc(tstart) q(1, :)];
end
writematrix(Q, 'lab2_pt8.csv');