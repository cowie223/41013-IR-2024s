igus = IGUSReBel;

% igus.model.jacob0([0, 0, 0, 0, 0, 0])
% M = [1 1 zeros(1,4)];

t = 10; %Time in seconnds
deltaT = 0.05;
steps = t/deltaT;
delta = 2*pi/steps;
epsilon = 0.1;
W = diag([1 1 1 0.1 0.1 01.]);





% allocate array data
mask = [1 1 1 0 0 0];
qMatrix = zeros(steps, 6);
qdot = zeros(steps, 6);
theta = zeros(3, steps);
x = zeros(3, steps);
positionError = zeros(3, steps);
angleError = zeros(3, steps);

%set up inital pose

s = lspb(0, 1, steps);
for k = 1:steps
    x(1,k) = (1-s(k))*0.15 + s(k)*0.15;
    x(2,k) = (1-s(k))*0.35 + s(k)*0.35;
    x(3,k) = 0.1 + 0.2*sin(k + delta);
    theta(1,k) = 0;
    theta(2,k) = 5*pi/9;
    theta(3,k) = 0;
end

% for i = 1:steps
%     % x(:,i) = [0.2*cos(deltaTheta*i) + 0.3*cos(deltaTheta*i)
%     %           0.2*sin(deltaTheta*i) + 0.3*cos(deltaTheta*i) 
%     %           0];
% 
%     x(:,i) = [(-0.25 + i*0.5/steps), 0.2, 0.2];
% 
% 
% end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];  
q0 = [0 0 0 0 0 0];

qMatrix(1,:) = igus.model.ikine(T, 'q0', [0 0 0 0 0 0], 'mask', [1 1 1 0 0 0], 'forceSoln')

qMatrix(abs(qMatrix) < 0.001) = 0;




for i = 1:steps-1
    T = igus.model.fkine(qMatrix(i,:)).T;

    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = igus.model.jacob0(qMatrix(i,:));             % Get Jacobian at current joint state
    m(i) = real(sqrt(det(J*J')));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < igus.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > igus.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

%plot results

tic

figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
igus.model.plot(qMatrix,'trail','r-')

disp(['Plot took ', num2str(toc), 'seconds'])

for i = 1:6
    figure(2)
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    ylabel('Angle (rad)')
    refline(0,igus.model.qlim(i,1));
    refline(0,igus.model.qlim(i,2));

    figure(3)
    subplot(3,2,i)
    plot(qdot(:,i),'k','LineWidth',1)
    title(['Joint ',num2str(i)]);
    ylabel('Velocity (rad/s)')
    refline(0,0)
end

figure(4)
subplot(2,1,1)
plot(positionError'*1000,'LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Position Error (mm)')
legend('X-Axis','Y-Axis','Z-Axis')

subplot(2,1,2)
plot(angleError','LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Angle Error (rad)')
legend('Roll','Pitch','Yaw')
figure(5)
plot(m,'k','LineWidth',1)
refline(0,epsilon)
title('Manipulability')

%     xdot = (x(:,i+1) - x(:,i))/deltaT                                    % Calculate velocity at discrete time step
%     J = igus.model.jacob0(qMatrix(i,:));                                            % Get the Jacobian at the current state
%     J = J(1:3,:);                                                           % Take only first 2 rows
%     m(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
%     if m(:,i) < minManipMeasure
%         qdot = inv(J'*J + 0.01*eye(6))*J'*xdot;
%     else
%         qdot = pinv(J) * xdot;                                               % Solve velocitities via RMRC
%     end
% 
%     error(:,i) = xdot - J*qdot;
%     qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';                         % Update next joint state
% 
% end
% 
% figure(1)
% set(gcf,'units','normalized','outerposition',[0 0 1 1])
% 
% igus.PlotAndColourRobot();
% for j = 1:steps
%     animate(igus.model, qMatrix(i,:));
% 
% end
% 
% 
% igus.model.plot(qMatrix,'trail','r-')                                               % Animate the robot
% figure(2)
% plot(m,'k','LineWidth',1);                                                  % Plot the Manipulability
% title('Manipulability of 2-Link Planar')
% ylabel('Manipulability')
% xlabel('Step')
% figure(3)
% plot(error','Linewidth',1)
% ylabel('Error (m/s)')
% xlabel('Step')
% legend('x-velocity','y-velocity');
% 
% 
% 
% 
% 
% 
