img = cam.getImage();
T_0_check = [0 1 0 100; 1 0 0 -50; 0 0 -1 0; 0 0 0 1];
maskedImg = mask_checker(img, cam);
imshow(maskedImg);
t = text(0, 0, '0', 'Color', 'red');

while true
    [x, y] = ginput(1);
    
    worldPt = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4)', [x y]);
    
    r_pos = inv(T_0_check) * [worldPt'; 0; 1];

    r_pos = centers_to_positions(r_pos(1:2)');
    t.Position = [x y];
    t.String = num2str([r_pos(1) r_pos(2)]);
end