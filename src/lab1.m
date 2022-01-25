%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

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
  viaPts = [45,0];
  
  tstart = tic;
  for k = viaPts
    robot.interpolate_jp([k 0 0], 3000);
    while toc(tstart) < 4
      measured = robot.measured_js(1,0);
      measured_values = [measured_values; measured(1, :)];
      times = [times; 1000*toc(tstart)];
    end
  end
  
  % Closes then opens the gripper
  robot.closeGripper()
  pause(1)
  robot.openGripper()

    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
end

figure();
subplot(3, 1, 1);
plot(times, measured_values(:, 1));
title("Joint 1 Position vs. Time for 45 Degree Turn");
xlabel("Time (ms)");
ylabel("Joint Position (degrees)");
subplot(3, 1, 2);
plot(times, measured_values(:, 2));
title("Joint 2 Position vs. Time for 45 Degree Turn");
xlabel("Time (ms)");
ylabel("Joint Position (degrees)");
ylim([-5,5]);
subplot(3, 1, 3);
plot(times, measured_values(:, 3));
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


combined = [times measured_values];
writematrix(combined, "45_deg_3s.csv")

% Clear up memory upon termination
robot.shutdown()
