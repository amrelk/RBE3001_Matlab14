classdef Vel_Planner
    methods
        function A = inv_velkine(self, p0, p1, J)
            U=[p1(1)-p0(1);
                p1(2)-p0(2);
                p1(3)-p0(3)];
            X=(U/norm(U))*10;
            Jp=J(1:3,:);
            A=Jp\X;
        end
    end
end