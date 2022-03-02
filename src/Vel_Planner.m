classdef Vel_Planner
    methods
        function A = inv_velkine(self, p0, p1, J, prop)
            U=[p1(1) - p0(1);
               p1(2) - p0(2);
               p1(3) - p0(3)];
            if nargin > 4 && prop
                X = U*10;
            else
                X = (U/norm(U))*300;
            end
            Jp = J(1:3,:);
            A = Jp\X;
        end

        function q = ik3001_numeric(self, p, model)
            kine = Kinematics();
            q = [0 0 0];
            qp = [0 0 0];
            while ~isequal(abs(p - qp) <= 0.5, [1 1 1])
                J = kine.jacob3001(q);
                qp = kine.fk3001(q);
                qp = qp(1:3, 4)';
                v = self.inv_velkine(qp, p, J, 1);
                q = q + v';
                if nargin > 2
                    model.plot_arm([q]);
                    view([0 1 0]);
                    pause(0.05);
                end
            end
        end
    end
end