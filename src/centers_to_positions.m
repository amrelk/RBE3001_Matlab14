function out = centers_to_positions(in, height)
    if nargin > 1
        r_ball = height;
    else
        r_ball = 10.5; %mm CHANGE THIS TO BE MORE ACCURATE
    end
    p_c = [395.84 24.56 108.67]; %magic numbers!!! (camera position)
    out = zeros(size(in));
    out = [out r_ball*ones(size(in, 1), 1)];
    for k = 1:size(in, 1)
        out(k, 1) = ( (in(k, 1) - p_c(1))*( 1-(r_ball / p_c(3)) ) ) + p_c(1);
        out(k, 2) = ( (in(k, 2) - p_c(2))*( 1-(r_ball / p_c(3)) ) ) + p_c(2);
    end
end