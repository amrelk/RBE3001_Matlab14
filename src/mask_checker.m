function out = mask_checker(in, cam)
    T_0_check = [0 1 0 100; 1 0 0 -50; 0 0 -1 0; 0 0 0 1];
    wbounds = [-125 135; -125 -135; 150 -135; 150 135];
    chbounds = transform_pts(wbounds, T_0_check);
    zCoord = zeros(size(chbounds,1),1);
    chbounds = [chbounds zCoord];
    imbounds = worldToImage(cam.cam_IS, cam.cam_pose(1:3, 1:3), cam.cam_pose(1:3, 4)', chbounds);
    mask = roipoly(in, imbounds(:, 1), imbounds(:, 2));
    out = bsxfun(@times, in, cast(mask, 'like', in));
end