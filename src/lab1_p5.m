%%
% RBE3001 - Laboratory 1 
%

clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs);
load('joints.mat')


try
    times = cell(1, 8);
    values = cell(1, 8);
    
    for k = 1:4
        robot.interpolate_jp([0 0 0], 1000);
        while ~robot.at_goal_js()
            pause(0.1);
        end
        pause(0.5);
        
        tstart = tic;
        robot.interpolate_jp(joint_values(k, :), 1000);
        while ~robot.at_goal_js()
            measured = robot.measured_js(1,0);
            values{k} = [values{k}; measured(1, :)]; %#ok<*SAGROW>
            times{k} = [times{k}; 1000*toc(tstart)];
        end
        
        robot.interpolate_jp([0 0 0], 1000);
        while ~robot.at_goal_js()
            pause(0.1);
        end
        pause(0.5);
        
        tstart = tic;
        robot.servo_jp(joint_values(k, :));
        while ~robot.at_goal_js()
            measured = robot.measured_js(1,0);
            values{k+4} = [values{k+4}; measured(1, :)];
            times{k+4} = [times{k+4}; 1000*toc(tstart)];
        end
    end
    
    robot.interpolate_jp([0 0 0], 1000);
    while ~robot.at_goal_js()
        pause(0.1);
    end
    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
end

j1values = padcat(values{1}(:,1), values{2}(:,1), values{3}(:,1));
j2values = padcat(values{1}(:,2), values{2}(:,2), values{3}(:,3));
j3values = padcat(values{1}(:,3), values{2}(:,3), values{3}(:,3));

[~, ind] = max(cellfun('size', times, 1)); times_m = times{ind};

for k = 1:4
    figure();
    tiledlayout(3, 1);
    ax1 = nexttile;
    plot(times{k}, values{k}(:, 1));
    hold on;
    plot(times{k+4}, values{k+4}(:, 1));
    hold off;
    title("Joint 1 Position vs. Time");
    xlabel("Time (ms)"); ylabel("Position (degrees)");
    legend('1000ms Interpolated', 'Servo');
    
    ax2 = nexttile;
    plot(times{k}, values{k}(:, 2));
    hold on;
    plot(times{k+4}, values{k+4}(:, 2));
    hold off;
    title("Joint 2 Position vs. Time");
    xlabel("Time (ms)"); ylabel("Position (degrees)");
    
    ax3 = nexttile;
    plot(times{k}, values{k}(:, 3));
    hold on;
    plot(times{k+4}, values{k+4}(:, 3));
    hold off;
    title("Joint 3 Position vs. Time");
    xlabel("Time (ms)"); ylabel("Position (degrees)");
    
    linkaxes([ax1 ax2 ax3], 'xy');
end
% 
% timedeltas = [];
% for el = 2:length(times)
%     timedeltas = [timedeltas; times(el) - times(el-1)];
% end
% figure();
% histogram(timedeltas, "BinLimits", [5, 100]);
% title('Time Delta Histogram Outliers');
% xlabel('Delta Time (ms)');
% ylabel('Count');
% figure();
% histogram(timedeltas, 30, "BinLimits", [0, 5]);
% title('Time Delta Histogram without Outliers');
% xlabel('Delta Time (ms)');
% ylabel('Count');
% 
% 
% combined = [times values{ind}];
% writematrix(combined, "../out/45_deg_3s.csv")

% Clear up memory upon termination
robot.shutdown()
