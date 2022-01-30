classdef Model
    properties
        kine = Kinematics();
    end
   
    methods    
        function plot_arm(self, q)
            t{1} = eye(4);
            t{2} = self.kine.fk3001_inter(q, 1);
            t{3} = self.kine.fk3001_inter(q, 2);
            t{4} = self.kine.fk3001_inter(q, 3); 
            t{5} = self.kine.fk3001(q);
            Q = [t{1}(1, 4) t{2}(1, 4) t{3}(1, 4) t{4}(1, 4) t{5}(1, 4);
                 t{1}(2, 4) t{2}(2, 4) t{3}(2, 4) t{4}(2, 4) t{5}(2, 4);
                 t{1}(3, 4) t{2}(3, 4) t{3}(3, 4) t{4}(3, 4) t{5}(3, 4)];
            plot3(Q(1,:), Q(2,:), Q(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', [0.5,0.5,0.5]);
            axis([-200 200 -200 200 0 400]);
            pbaspect([1 1 1]);
            grid on;
            hold on;
            axisColors = ['r', 'b', 'g'];
            for k = 1:5
                for a = 1:3
                    ta = [t{k}(1:3, 4)' ; t{k}(1:3, 4)' + 50*t{k}(1:3, a)'];
                    p = plot3(ta(:, 1), ta(:, 2), ta(:, 3));
                    p.Color = axisColors(a);
                    p.LineWidth = 2;
                end
            end
            hold off;
            title('3D Stick Model')
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
        end
    end
    
end