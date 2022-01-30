kine = Kinematics();

t = zeros(71540, 3);
n = 1;

for q1 = -180:5:180
    for q2 = -85:5:85
        for q3 = -50:5:85
            T = kine.fk3001([q1 q2 q3]);
            t(n, :) = T(1:3, 4);
            n = n+1;
        end
    end
end

k = boundary(t, 0);
trisurf(k,t(:,1),t(:,2),t(:,3),'FaceColor','red','FaceAlpha',1);
axis equal;