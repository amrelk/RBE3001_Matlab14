classdef Model
    properties
        kine = Kinematics();
    end
   
    methods    
        function plot_arm(self, q)
            t1 = self.kine.fk3001_inter(q, 1);
            t2 = self.kine.fk3001_inter(q, 2);
            t3 = self.kine.fk3001_inter(q, 3); 
            t4 = self.kine.fk3001(q); 
            Q = [0 t1(1, 4) t2(1, 4) t3(1, 4) t4(1, 4);
                0 t1(2, 4) t2(2, 4) t3(2, 4) t4(2, 4);
                0 t1(3, 4) t2(3, 4) t3(3, 4) t4(3, 4)]
            plot3(Q(1,:), Q(2,:), Q(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', [0.5,0.5,0.5]);
            axis([-200 200 -200 200 0 400])
            pbaspect([1 1 1]);
            grid on;
            title('3D Stick Model')
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
        end
    end
    
end