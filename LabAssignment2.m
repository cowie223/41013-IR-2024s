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

        function RMRC_7DOF(self, startPos, endPos)
            % Set parameters for the simulation
            % igus = IGUSReBel();  % Load  model
            % igus.model.base = transl(0, 0, 0);
            t = 10;             % Total time (s)
            deltaT = 0.02;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.3;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
        
            % Allocate array data for 7DOF
            qMatrix = zeros(steps,7);       % Array for joint angles
            qdot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            startRPY = tr2rpy(startPos);
            endRPY = tr2rpy(endPos);
        
        
            % Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i = 1:steps
                x(1,i) = (1-s(i))*startPos(1,4) + s(i)*endPos(1,4); % Points in x
                x(2,i) = (1-s(i))*startPos(2,4) + s(i)*endPos(2,4); % Points in y
                x(3,i) = (1-s(i))*startPos(3,4) + s(i)*endPos(3,4); % Points in z
                theta(1,i) = (1-s(i))*startRPY(1) + s(i)*endRPY(1);                  % Roll angle 
                theta(2,i) = (1-s(i))*startRPY(2) + s(i)*endRPY(2);            % Pitch angle
                theta(3,i) = (1-s(i))*startRPY(3) + s(i)*endRPY(3);                  % Yaw angle
            end
        
            T = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) x(:,1); zeros(1,3) 1];   % Create transformation of first point and angle
            q0 = zeros(1,7);                                                        % Initial guess for joint angles
            qMatrix(1,:) = self.igus.model.ikcon(T, q0);                                  % Solve joint angles to achieve first waypoint
        
            % Track the trajectory with RMRC
        
            % modify workspace
            hold on;
            grid on;
            axis equal;
            axis([-1 1 -1 1 -1 1]);
            view(3);
        
        
            for i = 1:steps-1
                % FK for the current joint state, access the 'T' field
                T = self.igus.model.fkine(qMatrix(i,:)).T;                                        
                deltaX = x(:,i+1) - T(1:3,4);                                       % Position error for the next waypoint
                Rd = rpy2r(theta(1,i+1), theta(2,i+1), theta(3,i+1));               % Next RPY angles as rotation matrix
                Ra = T(1:3,1:3);                                                    % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                        % Rotation matrix error
                S = Rdot * Ra';                                                     % Skew symmetric matrix
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2); S(1,3); S(2,1)];
                xdot = W * [linear_velocity; angular_velocity];                     % End-effector velocity
        
                % Jacobian and DLS Inverse
                J = self.igus.model.jacob0(qMatrix(i,:));                 
                m = sqrt(det(J*J'));  % Manipulability measure
                if m < epsilon
                    lambda = (1 - m/epsilon) * 5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda * eye(7)) * J';                            % DLS Inverse for 7DOF
                qdot(i,:) = (invJ * xdot)';                                         % Solve RMRC equation
        
                % Joint limits check for 7DOF
                for j = 1:7
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.igus.model.qlim(j,1) || ...
                       qMatrix(i,j) + deltaT*qdot(i,j) > self.igus.model.qlim(j,2)
                        qdot(i,j) = 0; % Stop motor if limits exceeded
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                   % Update joint state
                
                % update plot
                self.igus.model.animate(qMatrix(i,:));
                % plot end position
                drawnow();
                if(mod(i, 5) == 0)
                    plot3(T(1,4), T(2,4), T(3,4), 'r.', 'MarkerSize', 5);
                    drawnow();
                end
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
                        % ---------- RMRC ----------
            if(index == 7) % calls fucntion with start and end transform
                % RMRC4 file did not work when changin base position
                RMRC_7DOF(transl(0.3, 0.3, -0.2)*rpy2tr(pi, 0, 0), transl(0.3, 0.3, -0.2)*rpy2tr(pi, 0, 0));
            end
        end
    end
end