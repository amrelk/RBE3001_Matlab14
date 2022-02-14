classdef Kinematics
    properties
        L = [55 40 100 100];
    end
    methods
        function J = jacob3001(self, q)
            J = zeros(6, 3);
            pe = self.fk3001(q);
            pe = pe(1:3, 4);
            for i = 1:3
                Ti = self.fk3001_inter(q, i);
                zi = Ti(1:3, 3);
                pi = Ti(1:3, 4);
                J(:, i) = [cross(zi, pe - pi); zi];
            end
        end

        function pdot = fdk3001(self, q, qdot)
            if nargin == 2
                qdot = q(2, :);
                q = q(1, :);
            end
            J = self.jacob3001(q);
            qdot = deg2rad(qdot)';
            pdot = J*qdot;
        end

        function q = ik3001(self, p)
            q = zeros(1, 3);
            xc = p(1);
            yc = p(2);
            zc = p(3);
            s = zc - (self.L(1) + self.L(2));
            r = sqrt(xc^2+yc^2);
            q(1) = atan2d(sign(yc)*sqrt(1-(xc/r)^2), xc/r);
            D = (self.L(3)^2 + self.L(4)^2 - (r^2 + s^2))/(2*self.L(3)*self.L(4));
            q(3) = 90 - atan2d(sqrt(1-D^2), D);
            b = r/sqrt(r^2+s^2);
            a = (self.L(3)^2 + r^2 + s^2 - self.L(4)^2)/(2*self.L(3)*sqrt(r^2+s^2));
            q(2) = 90 - atan2d(sqrt(1-a^2), a) - atan2d(sign(s)*sqrt(1-b^2), b);
            if (abs(q(1)) > 90) || (q(2) > 95) || (q(2) < -45) || (q(3) > 60) || (q(3) < -90)
                error('not in acceptable joint space!');
            end
        end
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