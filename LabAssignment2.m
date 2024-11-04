classdef LabAssignment2 < handle
    % Lab Assignment 2 main class. Houses functions to call other classes
    % and functions, while maintaining logic and control sequencing
    % locally.

    % Developed by:     
    %                   Bernard Lyons (bernard.lyons@student.uts.edu.au)
    %                   Alex Sitkowski (alexander.e.sitkowski@student.uts.edu.au)
    %                   Luke Cowie (luke.cowie@student.uts.edu.au)

    properties
        igus;                                                               % Igus property for handling the IGUS bot
        dobot;                                                              % Dobot property for handling the Dobot Magician bot
        igusUse;                                                            % Property for storing the 'use' position of the IGUS bot
        igusCharge;                                                         % Property for storing the 'charge' position of the IGUS bot
        aboveUse;                                                           % Property for storing the safe 'above usage' position - for waypoint mapping
        aboveCharge;                                                        % Property for storing the safe 'above charge' position - for waypoint mapping
        dobotCharge;                                                        % Property for storing the 'charge' position of the Dobot
        belowCharge;                                                        % Property for storing the safe 'below charge' position - for waypoint mapping
        idleIgusPos;                                                        % Property for storing the 'idle' position of the IGUS bot
        idleDobotPos;                                                       % Property for storing the 'idle' position of the Dobot
        safeOut;                                                            % Property for handling the safety status of the class
        mainIndex;                                                          % Property for storing the index of which the main code should run
        arduinoDevice;                                                      % Property for storing the 'arduino' variable - used for hardware E-Stop
        
    end
    
    methods
        function self = LabAssignment2()
            % Initialisation function, to setup variables and values as
            % required for function of the class.
            
            igusUseLoc = [-0.75,0,1];                                       % Setup of default 'use' position for IGUS bot
            igusUsePose = rotx(pi)*rotz(-pi/2);                             % Determine rotation parameters, to avoid rotation where it is not wanted
            self.igusUse = SE3(igusUsePose,igusUseLoc);                     % Combine translation & rotation matrix to form translation matrix
            
            igusChargeLoc = [-1.2,-0.9,0.85];                               % Setup of default 'charge' position for IGUS bot
            igusChargePose = rotx(-pi/2)*rotz(0);
            self.igusCharge = SE3(igusChargePose,igusChargeLoc);
            
            aboveUseLoc = [-0.75,0,1.2];                                    % Setup of waypoint above use position, to avoid light curtains
            self.aboveUse = SE3(igusUsePose,aboveUseLoc);
                
            aboveChargeLoc = [-1.2,-0.75,1.2];                              % Setup of waypoint above charge position, to avoid light curtains
            self.aboveCharge = SE3(igusChargePose,aboveChargeLoc);
            
            dobotChargeLoc = [-1.11,-1,0.75];                               % Setup of default 'charge' position for Dobot
            dobotChargePose = rotx(pi/2)*rotz(0);
            self.dobotCharge = SE3(dobotChargePose,dobotChargeLoc);
            
            belowChargeLoc = [-1.11,-1,0.7];                                % Setup of waypoint below charge position, to allow proper connection of charger cable
            self.belowCharge = SE3(dobotChargePose,belowChargeLoc); 

            igusLoc = transl([-1.4,-1.35,1.03]);                            % Setup of IGUS bot base position
            self.igus = IGUSReBel(igusLoc);                                 % Plot IGUS bot using above position

            dobotLoc = transl([-1.4,-1.15,0.55]);                           % Setup of Dobot base position
            self.dobot = DobotMagician(dobotLoc);                           % Plot Dobot using above position

            igusIdleLoc = transl(self.igus.model.fkine(self.igus.model.getpos));    % Setup of default idle position for IGUS bot
            igusIdlePose = rotx(0)*rotz(0);
            self.idleIgusPos = SE3(igusIdlePose,igusIdleLoc);

            dobotIdleLoc = transl(self.dobot.model.fkine(self.dobot.model.getpos)); % Setup of default idle position for Dobot
            dobotIdlePose = rotx(0)*rotz(0);
            self.idleDobotPos = SE3(dobotIdlePose,dobotIdleLoc);

            self.initEnvironment();                                         % Call the environment initialisation function to setup the environment
            self.initEstop();                                               % Call the E-Stop initialisation function to setup communication with Arduino - for Hardware E-Stop
            self.safeOut = 0;                                               % Define initial state as safe, to allow main function to run
            % self.main();                                                    % Call main function
        end

        function initEnvironment(self)
            % Initialisation function, to setup the environment & place
            % robots for simulation purposes.

            axis([-2,2,-2,2,0,2.5]);                                        % Setup workspace for nice viewing
            hold on
            PlaceObject('completeEnvironment.ply',[0,0,0]);                 % Load environment .ply file. This can be split up into the separate .ply files if required
        end

        function initEstop(self)                                            
            % Initialisation function to facilitate communication with
            % the hardware Estop, running via an Arduino Uno.

            % REMEMBER TO CHANGE COM PORT IF PHYSICAL USB PORT IS CHANGED
            self.arduinoDevice = serialport("COM3", 9600);                  % Setup serial communication with Arduino via serial monitor. Match baud with config via IDE
            configureTerminator(self.arduinoDevice, "LF");
        end

        function sensorCall(self)                                           
            % Call function to gather and update sensor status'. Should be
            % used prior to referencing any sensor data in the class.

            % For future implementation
        end

        function moveBot(self,loc,index)
            % Function used to animate movement of the robot from a current
            % location to a defined end position. Uses inverse kinematics &
            % trapezoidal trajectory generation.

            steps = 100;                                                    % Resolution of trajectory
            if(index == 1)                                                  % Index == 1 --> IGUS, Index == 2 --> Dobot
                q0 = self.igus.model.getpos;                                % Determine current position (joint angles), and use as base position
    
                q1 = self.igus.model.ikcon(loc,q0);                         % Determine required joint angles via Inverse Kinematics
                s = lspb(0,1,steps);                                        % Trapezoidal trajectory generation
                qMatrix = nan(steps,7);                                     % Generate empty matrix of the adequate size
    
                for j = 1:steps                                             
                    qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
                end
    
                hwSafe0 = SafetyCall(self.arduinoDevice);                   % Check Hardware E-Stop state prior to any movement
            for j=1:1:steps                                                 % Animate movement through trajectory           
                hwSafe1 = SafetyCall(self.arduinoDevice);                   % Recompute Hardware E-Stop status at the start of each step
                if(hwSafe0 ~= hwSafe1)                                      % Compare inital status to current status (monitor change)
                    hwSafe0 = hwSafe1;                                      % Update status if it has changed
                    if(hwSafe0 == 1)
                        disp('Hardware E-Stop Pressed! System stopped.');
                    elseif(hwSafe0 == 0)
                        disp('Hardware E-Stop Reset! System may resume.');
                    end
                end

                if(self.safeOut == 0 && hwSafe0 == 0)                       % If the system is safe (both GUI & HW E-stops), continue with the animation step
                    newQ=qMatrix(j,:);
                    self.igus.model.animate(newQ);
                    pause(0.01);
                elseif(self.safeOut == 1 || hwSafe0 == 1)                   % If the system is unsafe (either GUI or HW E-stop), forfeit the animation step and stop all movement
                    break;
                end
            end

                % disp("Target Position: ");                                % Logging of position for debugging/accuracy check
                % disp(transl(loc));
                % disp("Actual Result: ");
                % disp(transl(self.igus.model.fkine(self.igus.model.getpos)));
                % disp("--------");

            elseif(index == 2)
                q0 = self.dobot.model.getpos;
    
                q1 = self.dobot.model.ikcon(loc,q0);                       
                s = lspb(0,1,steps);                                       
                qMatrix = nan(steps,5); 
    
                for j = 1:steps                                             
                    qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
                end
    
                hwSafe0 = SafetyCall(self.arduinoDevice);

                for j=1:1:steps                                                       
                    hwSafe1 = SafetyCall(self.arduinoDevice);
                    if(hwSafe0 ~= hwSafe1)
                        hwSafe0 = hwSafe1;
                        if(hwSafe0 == 1)
                            disp('Hardware E-Stop Pressed! System stopped.');
                        elseif(hwSafe0 == 0)
                            disp('Hardware E-Stop Reset! System may resume.');
                        end
                    end
                    
                    if(self.safeOut == 0 && hwSafe0 == 0)
                        newQ=qMatrix(j,:);
                        self.dobot.model.animate(newQ);
                        pause(0.01);
                    elseif(self.safeOut == 1 || hwSafe0 == 1)
                        break;
                    end
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
            axis([-2 2 -2 2 0 2.5]);
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

        function main(self)
            % Main function to call and run necessary code blocks in order
            % to execute class effectively.
                            
            % ---------- Move to Use Pos ----------    
            if(self.mainIndex == 1 && self.safeOut == 0)
                moveBot(self,self.aboveUse,1);                              % Move Igus above use position
                moveBot(self,self.igusUse,1);                               % Move Igus down to use position
                self.mainIndex = 0;                                         % Return to null step state
                pause(0.5);
            end
            % ---------- Move to Charge Pos ----------
            if(self.mainIndex == 2 && self.safeOut == 0)
                moveBot(self,self.aboveUse,1);                              % Move Igus above use position
                moveBot(self,self.aboveCharge,1);                           % Move Igus above charge position
                moveBot(self,self.igusCharge,1);                            % Move Igus down to charge position
                moveBot(self,self.belowCharge,2);                           % Move Dobot below charge position
                moveBot(self,self.dobotCharge,2);                           % Move Dobot up to charge position
                self.mainIndex = 0;
                pause(0.5);
            end
            % ---------- Move to Wake Pos ----------
            if(self.mainIndex == 3 && self.safeOut == 0)
                moveBot(self,self.belowCharge,2);                           % Move Dobot below charge position
                moveBot(self,self.aboveCharge,1);                           % Move Igus above charge position
                moveBot(self,self.aboveUse,1);                              % Move Igus above use position
                moveBot(self,self.igusUse,1);                               % Move Igus down to use position
                self.mainIndex = 0;
                pause(0.5);
            end
            % ---------- Return to Idle Pos ----------
            if(self.mainIndex == 4 && self.safeOut == 0)
                moveBot(self,self.aboveUse,1);                              % Move Igus above use position
                moveBot(self,self.idleIgusPos,1);                           % Move Igus to idle position
                moveBot(self,self.idleDobotPos,2);                          % Move Dobot to idle position
                self.mainIndex = 0;
                pause(0.5);
            end
            % ---------- E-STOP ----------
            if(self.mainIndex == 5 && self.safeOut ~= 1)                    % If E-Stop is not already active & is pressed, set safety state to 1 (unsafe)
                setSafety(self,1);
                self.mainIndex = 0;
            end
            % ---------- RESET ----------
            if(self.mainIndex == 6 && self.safeOut ~= 0)                    % If E-Stop is active and reset is pressed, set safety state to 0 (safe)
                setSafety(self,0);
                self.mainIndex = 0;
            end
            % ---------- RMRC ----------
            if(self.mainIndex == 7) % calls function with start and end transform
                % RMRC4 file did not work when changing base position
                RMRC_7DOF(self, transl(0.3, 0.3, -0.2)*rpy2tr(pi, 0, 0), transl(0.3, 0.3, -0.2)*rpy2tr(pi, 0, 0));
            end
            pause(0.1);

        end

        function self = setSafety(self,status)
            % Function to set the safety state of the program. 
            % State == 1 --> unsafe, state == 0 --> Safe

            if(status == 1)
                disp('E-Stop Pressed! System stopped.');
            elseif(status == 0)
                disp('Safety Reset! System may resume.');
            end
            self.safeOut = status;
            main(self);
        end

        function self = setIndex(self,index)
            % Function to set the index, to be used when running the main
            % function.

            self.mainIndex = index;
            main(self);
        end
    end
end