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
        igusUse;
        igusCharge;
        aboveUse;
        aboveCharge;
        dobotCharge;
        belowCharge;
        idleIgusPos;
        idleDobotPos;
        safeOut;
    end
    
    methods
        function self = LabAssignment2
            % Initialisation function, to setup variables and values as
            % required for function of the class.
            
            igusUseLoc = [-0.75,0,1];
            igusUsePose = rotx(pi)*rotz(-pi/2);
            self.igusUse = SE3(igusUsePose,igusUseLoc);
            
            igusChargeLoc = [-1.2,-0.9,0.85];
            igusChargePose = rotx(-pi/2)*rotz(0);
            self.igusCharge = SE3(igusChargePose,igusChargeLoc);
            
            aboveUseLoc = [-0.75,0,1.2];
            self.aboveUse = SE3(igusUsePose,aboveUseLoc);
            
            aboveChargeLoc = [-1.2,-0.75,1.2];
            self.aboveCharge = SE3(igusChargePose,aboveChargeLoc);
            
            dobotChargeLoc = [-1.11,-1,0.75];
            dobotChargePose = rotx(pi/2)*rotz(0);
            self.dobotCharge = SE3(dobotChargePose,dobotChargeLoc);
            
            belowChargeLoc = [-1.11,-1,0.7];
            self.belowCharge = SE3(dobotChargePose,belowChargeLoc); 

            igusLoc = transl([-1.4,-1.35,1.03]);
            self.igus = IGUSReBel(igusLoc);

            dobotLoc = transl([-1.4,-1.15,0.55]);
            self.dobot = DobotMagician(dobotLoc);

            igusIdleLoc = transl(self.igus.model.fkine(self.igus.model.getpos));
            igusIdlePose = rotx(0)*rotz(0);
            self.idleIgusPos = SE3(igusIdlePose,igusIdleLoc);

            dobotIdleLoc = transl(self.dobot.model.fkine(self.dobot.model.getpos));
            dobotIdlePose = rotx(0)*rotz(0);
            self.idleDobotPos = SE3(dobotIdlePose,dobotIdleLoc);

            self.initEnvironment();
            % self.initEstop();
            % self.main();
        end

        function initEnvironment(self)
            % Initialisation function, to setup the environment & place
            % robots for simulation purposes.

            axis([-2,2,-2,2,0,2.5]);
            hold on
            PlaceObject('completeEnvironment.ply',[0,0,0]);
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

        function moveBot(self,loc,index)

            steps = 100;
            if(index == 1)
                q0 = self.igus.model.getpos;
    
                q1 = self.igus.model.ikcon(loc,q0);                            % Determine required joint angles via Inverse Kinematics
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
                % disp("Target Position: ");
                % disp(transl(loc));
                % disp("Actual Result: ");
                % disp(transl(self.igus.model.fkine(self.igus.model.getpos)));
                % disp("--------");
                
            elseif(index == 2)
                q0 = self.dobot.model.getpos;
    
                q1 = self.dobot.model.ikcon(loc,q0);                           % Determine required joint angles via Inverse Kinematics
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
                % disp("Target Position: ");
                % disp(transl(loc));
                % disp("Actual Result: ");
                % disp(transl(self.dobot.model.fkine(self.dobot.model.getpos)));
                % disp("--------");
            end
        end

        function main(self,index)
            % Main function to call and run necessary code blocks in order
            % to execute class effectively.

            % ---------- Move to Use Pos ----------    
            if(index == 1)
                moveBot(self,self.aboveUse,1);                                  % Move Igus above use position
                moveBot(self,self.igusUse,1);                                   % Move Igus down to use position
                pause(0.5);
            end
            % ---------- Move to Charge Pos ----------
            if(index == 2)
                moveBot(self,self.aboveUse,1);                                  % Move Igus above use position
                moveBot(self,self.aboveCharge,1);                               % Move Igus above charge position
                moveBot(self,self.igusCharge,1);                                % Move Igus down to charge position
                moveBot(self,self.belowCharge,2);                               % Move Dobot below charge position
                moveBot(self,self.dobotCharge,2);                               % Move Dobot up to charge position
                pause(0.5);
            end
            % ---------- Move to Wake Pos ----------
            if(index == 3)
                moveBot(self,self.belowCharge,2);                               % Move Dobot below charge position
                moveBot(self,self.aboveCharge,1);                               % Move Igus above charge position
                moveBot(self,self.aboveUse,1);                                  % Move Igus above use position
                moveBot(self,self.igusUse,1);                                   % Move Igus down to use position
                pause(0.5);
            end
            % ---------- Return to Idle Pos ----------
            if(index == 4)
                moveBot(self,self.aboveUse,1);                                  % Move Igus above use position
                moveBot(self,self.idleIgusPos,1);                               % Move Igus to idle position
                moveBot(self,self.idleDobotPos,2);                              % Move Dobot to idle position
                pause(0.5);
            end
            % ---------- E STOP ----------
            if(index == 5)

            end
            % ---------- RESET ----------
            if(index == 6)

            end
        end
    end
end