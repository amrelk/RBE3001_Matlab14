function out = rgb_to_color(rgb)
    hsv = rgb2hsv(rgb);
    out = 'unknown';
    if hsv(1) > 0.183 && hsv(1) < 0.296
        out = 'green';
    elseif hsv(1) > 0.112 && hsv(1) < 0.156
        out = 'yellow';
    elseif hsv(1) > 0.042 && hsv(1) < 0.109
        out = 'orange';
    elseif hsv(1) > 0.8 || hsv(1) < 0.05
        out = 'red';
    end
end