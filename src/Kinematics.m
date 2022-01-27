classdef Kinematics
    properties
        L = [55 40 100 100];
    end
    methods
        function T = dh2mat(self, dh)
            T = self.trotz(dh(1)) * self.ttransz(dh(2)) * self.ttransx(dh(3)) * self.trotx(dh(4));
        end
        function T = dh2fk(self, dh)
            n = size(dh);
            n = n(1);
            T = eye(4);
            for k = 1:n
                T = T * self.dh2mat(dh(k, :));
            end
        end
        function T = fk3001(self, q)
            T = self.dh2fk([ ...
                0 self.L(1) 0 0; ...
                q(1)-pi/2 self.L(2) 0 -pi/2; ...
                q(2) 0 self.L(3) 0; ...
                q(3)+pi/2 0 self.L(4) 0 ...
                ]);
        end
    end
    methods(Static)
        function T = trotx(theta)
            T = [1 0 0 0; 0 cos(theta) -sin(theta) 0; 0 sin(theta) cos(theta) 0; 0 0 0 1];
        end

        function T = trotz(theta)
            T = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
        end

        function T = ttransz(d)
            T = eye(4);
            T(3, 4) = d;
        end

        function T = ttransx(d)
            T = eye(4);
            T(1, 4) = d;
        end
    end
end