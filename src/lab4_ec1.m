planner = Vel_Planner();
kine = Kinematics();

targets = [100 0 195; 50 100 100; 100 50 50; 100 0 195];

q = [];
jank = [];

robot.servo_jp([0 0 0]);
while ~robot.at_goal_js()
    pause(0.1);
end
c = [];
tstart = tic;
ts = tstart;
cp = [0 0 0];
for n = 2:length(targets)
    while ~isequal(abs(cp - targets(n, :)) <= 2, [1 1 1])
        cp = robot.measured_cp(1);
        js = robot.measured_js();
        J = kine.jacob3001(js);
        vels = rad2deg(planner.inv_velkine(cp, targets(n, :), J)');
        robot.servo_jp(js + vels*toc(ts));
        disp(vels*toc(ts));
        ts = tic;
        c = [c; cp];
        pause(0.03);
    end
end