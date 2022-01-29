classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        SERV_ID = 1848;            % we will be talking to server ID 1848 on the Nucleo
        joints = [0 0 0];
        kine = Kinematics();
    end
    
    methods
        function x = at_goal_js(self)
            pos = self.measured_js(1,0);
            pos = pos(1,:);
            x = isequal(abs(pos - self.goal_js()) <= 1.8, [1 1 1]);
        end
        
        function x = goal_js(self)
            x = self.joints;
        end
        
        function T = goal_cp(self)
            q = self.goal_js();
            T = self.kine.fk3001(q);
        end
        
        %Set joint positions with interpolation
        function interpolate_jp(self, joints, time)
            self.joints = joints;
            self.write(1848, [time 0 joints]);
        end
        
        function x = measured_js(self, getpos, getvel)
            pos = [0 0 0];
            vel = [0 0 0];
            if getpos
                pos = self.read(1910);
                pos = pos([3 5 7])';
            end
            if getvel
                vel = self.read(1822);
                vel = vel([3 6 9])';
            end
            x = [pos;vel];
        end
        
        function T = measured_cp(self)
            q = self.measured_js(1, 0);
            q = q(1, :);
            T = self.kine.fk3001(q);
        end
       
        function x = setpoint_js(self)
            x = self.read(1910);
            x = x([2 4 6]);
        end
        
        function T = setpoint_cp(self)
            q = self.setpoint_js();
            T = self.kine.fk3001(q);
        end
        
        function servo_jp(self, joints)
            self.interpolate_jp(joints, 0);
        end
       
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
             self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end
    end
end
