classdef LabAssignment2
    % Lab Assignment 2 main class. Houses functions to call other classes
    % and functions, while maintaining logic and control sequencing
    % locally.

    % Developed by:     
    %                   Bernard Lyons (bernard.lyons@student.uts.edu.au)
    %                   Alex Sitkowski (alexander.e.sitkowski@student.uts.edu.au)
    %                   Luke Cowie (luke.cowie@student.uts.edu.au)

    properties
        igus;
        dobot;
        useLoc;
        chargeLoc;
        idleIgusPos;
        idleDobotPos;
        safeOut;

        flagIdle = true;
    end

    % methods (Static)
    %     function useCall(useGUI)
    %         % Function to allow data from GUI to be received
    % 
    %         self.useLoc = useGUI;
    %     end
    % 
    %     function chargeCall(chargeGUI)
    %         % Function to allow data from GUI to be received
    % 
    %         self.chargeLoc = chargeGUI;
    %     end
    % 
    %     function start()
    %         self.flagIdle = false;
    %         moveBot(self.igus,self.useLoc,7);
    % 
    %     end
    % 
    %     function sleep()
    %         self.flagIdle = false;
    %         moveBot(self.igus,self.chargeLoc,7);
    %         moveBot(self.dobot,self.chargeLoc,5);
    % 
    %     end
    % 
    %     function wake()
    %         self.flagIdle = false;
    %         moveBot(self.dobot,self.idleDobotPos,5);
    %         moveBot(self.igus,self.useLoc,7);
    % 
    %     end
    % 
    %     function idle()
    %         self.flagIdle = false;
    %         moveBot(self.igus,self.idleIgusPos,7);
    %         moveBot(self.dobot,self.idleDobotPos,5);
    % 
    %     end
    % end
    
    methods
        function self = LabAssignment2
            % Initialisation function, to setup variables and values as
            % required for function of the class.
            
            igusUseLoc = [-0.75,0,1];
            igusUsePose = rotx(pi)*rotz(-pi/2);
            self.useLoc = SE3(igusUsePose,igusUseLoc);

            igusChargeLoc = [-1.2,-0.9,0.85];
            igusChargePose = rotx(-pi/2)*rotz(0);
            self.chargeLoc = SE3(igusChargePose,igusChargeLoc); 

            self.initEnvironment();
            pause(2);
            % self.initEstop();
            self.main();
        end

        function initEnvironment(self)
            % Initialisation function, to setup the environment & place
            % robots for simulation purposes.

            axis([-2,2,-2,2,0,2.5]);
            hold on
            PlaceObject('completeEnvironment.ply',[0,0,0]);

            igusLoc = transl([-1.4,-1.35,1.03]);
            self.igus = IGUSReBel(igusLoc);

            dobotLoc = transl([-1.4,-1.15,0.55]);
            self.dobot = DobotMagician(dobotLoc);
            
            self.idleIgusPos = self.igus.model.getpos;
            self.idleDobotPos = self.dobot.model.getpos;
        end

        function initEstop(self)
            % Initialisation function to facilitate communication with
            % the hardware Estop, running via an Arduino Uno.

            % REMEMBER TO CHANGE COM PORT IF PHYSICAL USB PORT IS CHANGED
            arduinoDevice = serialport("COM3", 9600);  
            configureTerminator(arduinoDevice, "LF");
        end

        function sensorCall(self)
            % Call function to gather and update sensor status'. Should be
            % used prior to referencing any sensor data in the class.

        end

        function moveBot(self,loc)

            steps = 100;
            index = 1;
            if(index == 1)
                q0 = self.igus.model.getpos;
    
                q1 = self.igus.model.ikcon(loc,q0);                               % Determine required joint angles via Inverse Kinematics
                s = lspb(0,1,steps);                                           % Trapezoidal trajectory generation
                qMatrix = nan(steps,7);
    
                for j = 1:steps                                             
                    qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
                end
    
                for j=1:1:steps                                                % Animate movement through trajectory           
                    newQ=qMatrix(j,:);
                    self.igus.model.animate(newQ);
                    pause(0.01);
                end
                
            elseif(index == 2)
                q0 = self.dobot.model.getpos;
    
                q1 = self.dobot.model.ikcon(pose,q0);                               % Determine required joint angles via Inverse Kinematics
                s = lspb(0,1,steps);                                           % Trapezoidal trajectory generation
                qMatrix = nan(steps,5);
    
                for j = 1:steps                                             
                    qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
                end
    
                for j=1:1:steps                                                % Animate movement through trajectory           
                    newQ=qMatrix(j,:);
                    self.dobot.model.animate(newQ);
                    pause(0.01);
                end
            end
        end

        function main(self)
            % Main function to call and run necessary code blocks in order
            % to execute class effectively.

            % ---------- Move to Use Pos ----------
            moveBot(self,self.useLoc);

            % ---------- Move to Charge Pos ----------
            moveBot(self.igus,self.chargeLoc,7);
            moveBot(self.dobot,self.chargeLoc,5);

            % ---------- Move to Wake Pos ----------
            moveBot(self.dobot,self.idleDobotPos,5);
            moveBot(self.igus,self.useLoc,7);
            
            % ---------- Return to Idle Pos ----------
            moveBot(self.igus,self.idleIgusPos,7);
            moveBot(self.dobot,self.idleDobotPos,5);
        end
    end
end