T_0_check = [0 1 0 100; 1 0 0 -50; 0 0 -1 0; 0 0 0 1];
planner = Traj_Planner;
while true
    img = mask_checker(cam.getImage, cam);
    mask = maskBalls(img);
    erse = strel('square', 6);
    dise = strel('square', 6);
    eroded = imerode(mask, erse);
    dilated = imdilate(eroded, dise);
    blurred = imgaussfilt(img, 10);
    masked = bsxfun(@times, img, cast(dilated, 'like', img));
    [circles, radii, metric] = imfindcircles(masked, [18 30], 'Sensitivity', 0.97, 'ObjectPolarity', 'bright');
    imshow(img);
    hold on;
    if ~isempty(circles)
        scatter(circles(:, 1), circles(:, 2), metric*300);
        for k = 1:size(circles, 1)
            color = blurred(floor(circles(k, 2)), floor(circles(k, 1)), :);
            text(circles(k, 1), circles(k, 2), rgb_to_color(color), 'Color', 'r');
        end
        chkpt = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4)', circles(1, :));
        worldpt = transform_pts(chkpt, inv(T_0_check));
        worldpt = centers_to_positions(worldpt);
        try
            if isequal(rgb_to_color(blurred(floor(circles(1, 2)), floor(circles(1, 1)), :)), 'yellow')
                robot.servo_cp(worldpt + [0 0 50]);
            end
        catch e
            robot.interpolate_cp([100 0 20], 0.5);
            pause(0.5)
            robot.openGripper();
            disp(e);
        end
    end
    hold off;
    %pause(0.1);
end