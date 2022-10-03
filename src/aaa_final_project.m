% T_0_check = [0 1 0 92; 1 0 0 -92; 0 0 -1 0; 0 0 0 1];
R = [0 1 0;1 0 0;0 0 -1];
T_0_check = [R, [92; -75; 0];zeros(1,3), 1]; % 22
% T_0_check = [R, [100; -55; 0];zeros(1,3), 1]; % 27

planner = Traj_Planner;
while true
    img = mask_checker(cam.getImage, cam); % get checkerboard image
    mask = maskBalls(img); % mask out all the balls
    erse = strel('square', 6); % erosion size = 6px
    dise = strel('square', 6); % dilation size = 6px
    eroded = imerode(mask, erse); % erode image
    dilated = imdilate(eroded, dise); % dilate image
    blurred = imgaussfilt(img, 10); % blur image
    masked = bsxfun(@times, img, cast(dilated, 'like', img)); % apply eroded-dilated mask to image
    [circles, radii, metric] = imfindcircles(masked, [18 30], 'Sensitivity', 0.97, 'ObjectPolarity', 'bright');
    imshow(img);
    hold on;
    if ~isempty(circles)
        scatter(circles(:, 1), circles(:, 2), metric*300); % plot circles
        for k = 1:size(circles, 1)
            color = blurred(floor(circles(k, 2)), floor(circles(k, 1)), :);
            text(circles(k, 1), circles(k, 2), rgb_to_color(color), 'Color', 'r');
        end
        chkpt = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4)', circles(1, :));
        worldpt = transform_pts(chkpt, inv(T_0_check));
        worldpt = centers_to_positions(worldpt);
        try
            if true %isequal(rgb_to_color(blurred(floor(circles(1, 2)), floor(circles(1, 1)), :)), 'yellow')
                robot.openGripper();
                pause(0.3)
                moveToGrab(robot, worldpt);
                robot.closeGripper();
                pause(0.3);
                moveToPlace(robot, rgb_to_color(blurred(floor(circles(1, 2)), floor(circles(1, 1)), :)));
            end
        catch e
            robot.interpolate_cp([100 0 20], 0.5);
            pause(0.5)
            robot.openGripper();
            disp(e);
        end
    end
    hold off;
    pause(0.1);
end

function moveToPlace(robot, color)
    worldpt = color_to_drop(color);
    planner = Traj_Planner;
    pos = robot.measured_cp(true);
    med_pt = [(pos(1)+worldpt(1))/2, (pos(2)+worldpt(2))/2, 100];
    if (med_pt(1) < 75)
        med_pt(1) = 75;
    end
    traj = planner.xyz_traj(robot.measured_cp(true), med_pt, 0, 1);
    traj_st = tic;
    while (toc(traj_st) < 1)
        robot.servo_cp(traj_eval(traj, toc(traj_st)), true);
        pause(0.05);
    end
    traj = planner.xyz_traj(robot.measured_cp(true), worldpt + [0 0 15], 0, 1);
    robot.run_trajectory(traj', 1);
    robot.openGripper();
    pause(0.2);
    med_pt = [worldpt(1)*0.8, worldpt(2)*0.8, 60];
    traj = planner.xyz_traj(robot.measured_cp(true), med_pt, 0, 1);
    robot.run_trajectory(traj', 1);
end

function moveToGrab(robot, worldpt)
    planner = Traj_Planner;
    pos = robot.measured_cp(true);
    med_pt = [(pos(1)+worldpt(1))/2, (pos(2)+worldpt(2))/2, 70];
    if (med_pt(1) < 75)
        med_pt(1) = 75;
    end
    traj = planner.xyz_traj(robot.measured_cp(true), med_pt, 0, 0.5);
    robot.run_trajectory(traj', 0.5);
    traj = planner.xyz_traj(med_pt, worldpt + [0 0 40], 0, 0.5);
    traj_st = tic;
    while (toc(traj_st) < 0.5)
        robot.servo_cp(traj_eval(traj, toc(traj_st)), true);
        pause(0.05);
    end
    traj = planner.xyz_traj(worldpt + [0 0 40], worldpt, 0, 1);
    traj_st = tic;
    while (toc(traj_st) < 1)
        robot.servo_cp(traj_eval(traj, toc(traj_st)), true);
        pause(0.05);
    end
end

function p = traj_eval(traj, t)
    p = [polyval(traj(:, 1), t) polyval(traj(:, 2), t) polyval(traj(:, 3), t)];
end

function p = color_to_drop(color)
    if isequal(color, 'red')
        p = [0 150 0];
    elseif isequal(color, 'yellow')
        p = [20 -150 0];
    elseif isequal(color, 'green')
        p = [90 -140 0];
    elseif isequal(color, 'orange')
        p = [90 140 0];
    else
        p = [100 0 12];
    end
end