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
        function T = fk3001_inter(self, q, j)
            dh = [ ...
                0 self.L(1) 0 0; ...       % 0-1
                q(1) self.L(2) 0 -90; ...  % 1-2
                q(2)-90 0 self.L(3) 0; ... % 2-3
                q(3)+90 0 self.L(4) 0 ...  % 3-4
                ];
            T = self.dh2fk(dh(1:j, :));
        end
        function T = fk3001(~, q)
            T = [ cosd(q(1))*cosd(q(2) - 90)*cosd(q(3) + 90) - cosd(q(1))*sind(q(2) - 90)*sind(q(3) + 90), - cosd(q(1))*cosd(q(2) - 90)*sind(q(3) + 90) - cosd(q(1))*cosd(q(3) + 90)*sind(q(2) - 90), -sind(q(1)), 100*cosd(q(1))*cosd(q(2) - 90) + 100*cosd(q(1))*cosd(q(2) - 90)*cosd(q(3) + 90) - 100*cosd(q(1))*sind(q(2) - 90)*sind(q(3) + 90);
                  sind(q(1))*cosd(q(2) - 90)*cosd(q(3) + 90) - sind(q(1))*sind(q(2) - 90)*sind(q(3) + 90), - sind(q(1))*cosd(q(2) - 90)*sind(q(3) + 90) - sind(q(1))*cosd(q(3) + 90)*sind(q(2) - 90), cosd(q(1)), 100*sind(q(1))*cosd(q(2) - 90) + 100*sind(q(1))*cosd(q(2) - 90)*cosd(q(3) + 90) - 100*sind(q(1))*sind(q(2) - 90)*sind(q(3) + 90);
                  - cosd(q(2) - 90)*sind(q(3) + 90) - cosd(q(3) + 90)*sind(q(2) - 90), sind(q(2) - 90)*sind(q(3) + 90) - cosd(q(2) - 90)*cosd(q(3) + 90), 0, 95 - 100*cosd(q(2) - 90)*sind(q(3) + 90) - 100*cosd(q(3) + 90)*sind(q(2) - 90) - 100*sind(q(2) - 90);
                  0, 0, 0, 1];
        end
    end
    methods(Static)
        function T = trotx(theta)
            T = [1 0 0 0; 0 cosd(theta) -sind(theta) 0; 0 sind(theta) cosd(theta) 0; 0 0 0 1];
        end

        function T = trotz(theta)
            T = [cosd(theta) -sind(theta) 0 0; sind(theta) cosd(theta) 0 0; 0 0 1 0; 0 0 0 1];
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