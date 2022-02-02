kine = Kinematics();

t = zeros(80000, 3);
n = 1;

for q1 = linspace(-90, 90, 50)
    for q2 = linspace(-50, 98, 40)
        for q3 = linspace(-105, 74, 40)
            T = kine.fk3001([q1 q2 q3]);
            t(n, :) = T(1:3, 4);
            n = n+1;
        end
    end
end

k = boundary(t, 1);
trisurf(k,t(:,1),t(:,2),t(:,3),'FaceColor','red','FaceAlpha',1);
axis equal;