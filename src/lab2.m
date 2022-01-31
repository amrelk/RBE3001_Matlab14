model = Model();

pts = [0 0 0; 30 20 50; -20 10 60; 50 3 -45];
n = 1;

robot.servo_jp([0 0 0]);
while 1
    if robot.at_goal_js()
        robot.interpolate_jp(pts(n, :), 1000);
        if n < length(pts)
            n = n+1;
        else
            n = 1;
        end
    else
        q = robot.measured_js(1, 0);
        model.plot_arm(q(1,:));
    end
    pause(0.05);
end