T_0_check = [0 1 0 100; 1 0 0 -50; 0 0 -1 0; 0 0 0 1];
planner = Traj_Planner;
while true
    img = mask_checker(cam.getImage, cam);
    mask = maskObject(img);
    erse = strel('square', 3);
    dise = strel('square', 4);
    eroded = imerode(mask, erse);
    dilated = imdilate(eroded, dise);
    blurred = imgaussfilt(img, 10);
    masked = bsxfun(@times, img, cast(dilated, 'like', img));
    props = regionprops(dilated);
    centers = cell2mat({props(:).Centroid}');
    [~, ind] = max([props.Area]);
    circles = centers(ind, :);
    imshow(masked);
    hold on;
    if ~isempty(circles)
        scatter(circles(:, 1), circles(:, 2), 30);
        for k = 1:size(circles, 1)
            color = blurred(floor(circles(k, 2)), floor(circles(k, 1)), :);
            text(circles(k, 1), circles(k, 2), rgb_to_color(color), 'Color', 'r');
        end
        chkpt = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4)', circles(1, :));
        worldpt = transform_pts(chkpt, inv(T_0_check));
        worldpt = centers_to_positions(worldpt, 12);
        try
            if true %isequal(rgb_to_color(blurred(floor(circles(1, 2)), floor(circles(1, 1)), :)), 'yellow')
                robot.openGripper();
                pause(0.3)
                moveToGrab(robot, worldpt);
                robot.closeGripper();
                pause(0.3);
                moveToPlace(robot, rgb_to_color(blurred(floor(circles(1, 2)), floor(circles(1, 1)), :)));
                pause(0.3);
                robot.openGripper();
                pause(1);
            end
        catch e
            disp(e);
        end
    end
    hold off;
    pause(0.5);
end

function moveToPlace(robot, color)
    worldpt = color_to_drop(color);
    planner = Traj_Planner;
    robot.servo_jp([0 0 0]);
    while ~robot.at_goal_js()
        pause(0.1);
    end
    traj = planner.xyz_traj(robot.measured_cp(true), worldpt + [0 0 25], 0, 1);
    disp(traj_eval(traj, 0));
    traj_st = tic;
    while (toc(traj_st) < 1)
        robot.servo_cp(traj_eval(traj, toc(traj_st)));
        pause(0.05);
    end
end

function moveToGrab(robot, worldpt)
    planner = Traj_Planner;
    robot.servo_jp([0 0 0]);
    while ~robot.at_goal_js()
        pause(0.1);
    end
    traj = planner.xyz_traj(robot.measured_cp(true), worldpt + [0 0 30], 0, 1);
    disp(traj_eval(traj, 0));
    traj_st = tic;
    while (toc(traj_st) < 1)
        robot.servo_cp(traj_eval(traj, toc(traj_st)), true);
        pause(0.05);
    end
    traj = planner.xyz_traj(robot.measured_cp(true), worldpt, 0, 1);
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
        p = [0 -150 0];
    elseif isequal(color, 'green')
        p = [90 -150 0];
    elseif isequal(color, 'orange')
        p = [90 150 0];
    else
        p = [100, (rand()-0.5)*200, 40];
    end
end