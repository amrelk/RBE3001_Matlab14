classdef Traj_Planner
    methods
        function A = xyz_traj(self, p0, p1, t0, t1)
            A = [self.cubic_traj(p0(1), p1(1), 0, 0, t0, t1), ...
                 self.cubic_traj(p0(2), p1(2), 0, 0, t0, t1), ...
                 self.cubic_traj(p0(3), p1(3), 0, 0, t0, t1)];
        end
        function A = cubic_traj(self, p0, p1, v0, v1, t0, t1)
            M=[1 t0 t0^2 t0^3;
               0 1 2*t0 3*t0^2;
               1 t1 t1^2 t1^3;
               0 1 2*t1 3*t1^2];
            Q=[p0;
               v0;
               p1;
               v1];
            A=flip(M\Q);
        end

        function B = quintic_traj(self, p0, p1, v0, v1, a0, a1, t0, t1)
            M=[1 t0 t0^2 t0^3 t0^4 t0^5;
               0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
               0 0 2 6*t0 12*t0^2 20*t0^3;
               1 t1 t1^2 t1^3 t1^4 t1^5;
               0 1 2*t1 3*t1^2 4*t1^3 5*t1^4;
               0 0 2 6*t1 12*t1^2 20*t1^3];
            Q=[p0;
               v0;
               a0;
               p1;
               v1;
               a1];
            B=flip(M\Q);
        end
    end

end