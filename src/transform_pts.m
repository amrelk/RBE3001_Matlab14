function out = transform_pts(in, T)
    out = zeros(size(in));
    for k = 1:size(in, 1)
        t = T * [in(k, :)'; 0; 1];
        out(k, :) = [t(1) t(2)];
    end
end