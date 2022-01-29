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
try
    measured_values = [];
    times = [];
    robot.interpolate_jp([0 0 0], 1000);
    while ~robot.at_goal_js()
        pause(0.1);
    end
    
    trials = 3;
    times = cell(1, trials);
    values = cell(1, trials);
    for k = 1:trials
        pause(0.5);
        tstart = tic;
        robot.interpolate_jp([45 0 0], 2000);
        while ~robot.at_goal_js()
            measured = robot.measured_js(1,0);
            values{k} = [values{k}; measured(1, :)]; %#ok<*AGROW>
            times{k} = [times{k}; 1000*toc(tstart)];
        end
        robot.interpolate_jp([0 0 0], 1000);
        while ~robot.at_goal_js()
            pause(0.1);
        end
    end
    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
end

j1values = padcat(values{1}(:,1), values{2}(:,1), values{3}(:,1));
j2values = padcat(values{1}(:,2), values{2}(:,2), values{3}(:,3));
j3values = padcat(values{1}(:,3), values{2}(:,3), values{3}(:,3));

[~, ind] = max(cellfun('size', times, 1));
times = times{ind};

figure();
subplot(3, 1, 1);
plot(times, j1values);
title("Joint 1 Position vs. Time for 45 Degree Turn");
xlabel("Time (ms)");
ylabel("Joint Position (degrees)");
legend(["Trial 1", "Trial 2", "Trial 3"], "Location", "southeast");
subplot(3, 1, 2);
plot(times, j2values);
title("Joint 2 Position vs. Time for 45 Degree Turn");
xlabel("Time (ms)");
ylabel("Joint Position (degrees)");
ylim([-5,5]);
subplot(3, 1, 3);
plot(times, j3values);
title("Joint 3 Position vs. Time for 45 Degree Turn");
xlabel("Time (ms)");
ylabel("Joint Position (degrees)");
ylim([-5,5]);

timedeltas = [];
for el = 2:length(times)
    timedeltas = [timedeltas; times(el) - times(el-1)];
end
figure();
histogram(timedeltas, "BinLimits", [5, 100]);
title('Time Delta Histogram Outliers');
xlabel('Delta Time (ms)');
ylabel('Count');
figure();
histogram(timedeltas, 30, "BinLimits", [0, 5]);
title('Time Delta Histogram without Outliers');
xlabel('Delta Time (ms)');
ylabel('Count');


combined = [times values{ind}];
writematrix(combined, "../out/45_deg_3s.csv")

% Clear up memory upon termination
robot.shutdown()
