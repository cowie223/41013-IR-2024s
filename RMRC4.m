% Resolved Motion Rate Control in 7DOF
clf;
RMRC_7DOF(transl(0.3, 0.3, -0.2)*rpy2tr(pi, 0, 0), transl(0.3, 0.3, -0.2)*rpy2tr(pi, 0, 0));

function RMRC_7DOF(startPos, endPos)
    % 1.1) Set parameters for the simulation
    igus = IGUSReBel();  % Load 7DOF robot model (Assuming a 7DOF model is defined)
    % igus.model.base = transl(0, 0, 0);
    t = 10;             % Total time (s)
    deltaT = 0.02;      % Control frequency
    steps = t/deltaT;   % No. of steps for simulation
    delta = 2*pi/steps; % Small angle change
    epsilon = 0.3;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

    % 1.2) Allocate array data for 7DOF
    qMatrix = zeros(steps,7);       % Array for joint angles
    qdot = zeros(steps,7);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    startRPY = tr2rpy(startPos);
    endRPY = tr2rpy(endPos);


    % 1.3) Set up trajectory, initial pose
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
    qMatrix(1,:) = igus.model.ikcon(T, q0);                                  % Solve joint angles to achieve first waypoint

    % 1.4) Track the trajectory with RMRC

    
    hold on;
    grid on;
    axis equal;
    axis([-1 1 -1 1 -1 1]);
    view(3);


    for i = 1:steps-1
        % FK for the current joint state, access the 'T' field
        T = igus.model.fkine(qMatrix(i,:)).T;                                        
        deltaX = x(:,i+1) - T(1:3,4);                                       % Position error for the next waypoint
        Rd = rpy2r(theta(1,i+1), theta(2,i+1), theta(3,i+1));               % Next RPY angles as rotation matrix
        Ra = T(1:3,1:3);                                                    % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                        % Rotation matrix error
        S = Rdot * Ra';                                                     % Skew symmetric matrix
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2); S(1,3); S(2,1)];
        xdot = W * [linear_velocity; angular_velocity];                     % End-effector velocity

        % Jacobian and DLS Inverse
        J = igus.model.jacob0(qMatrix(i,:));                 
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
            if qMatrix(i,j) + deltaT*qdot(i,j) < igus.model.qlim(j,1) || ...
               qMatrix(i,j) + deltaT*qdot(i,j) > igus.model.qlim(j,2)
                qdot(i,j) = 0; % Stop motor if limits exceeded
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                   % Update joint state

        igus.model.animate(qMatrix(i,:));
        
        drawnow();
        if(mod(i, 1) == 0)
            plot3(T(1,4), T(2,4), T(3,4), 'r.', 'MarkerSize', 5);
            drawnow();
        end
    end


end

