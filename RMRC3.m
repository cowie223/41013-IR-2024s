function RMRC3(q0,q1,q2,igus)

    % igus = IGUSReBel;
    
    igus.model.jacob0([0, 0, 0, 0, 0, 0, 0]);
    
    M = [1 1 zeros(1,4)];
    
    deltaT = 0.05;
    
    minManipMeasure = 0.1;
    minManipMeasure = 0.1;
    steps = 200;
    deltaTheta = 2*pi/steps;
    x = zeros(3, steps);
    theta = zeros(3, steps);
    qMatrix = zeros(steps, 7);

    for i = 1:steps
        %parametric equatios for end effector path
        x(1,i) = (0.2 + i*0.2/steps);
        x(2,i) = -0.2 + 0.2*i/steps;
        x(3,i) = -0.05 + 0.02*i/steps;
        theta(1, i) = deg2rad(0);
        theta(2, i) = deg2rad(-90);
        theta(3, i) = deg2rad(0);


    end

    % q0 = [0, deg2rad(-95), deg2rad(54.1), deg2rad(-81), deg2rad(-13.1), deg2rad(-34.3), deg2rad(0)];
    %T = [eye(3) [x(:,1)];zeros(1,3) 1];
    T = [rpy2r(theta(1,1), theta(2,1), theta(3,1)), x(:,1);zeros(1,3), 1]

    % qMatrix(1,:) = igus.model.ikine(T, 'q0', q0, 'mask', [1 1 1 1 1 1], 'forceSoln');
    qMatrix(1,:) = igus.model.ikcon(T, zeros(1, 7));

    startPose = true;
    if startPose
        m = zeros(1,steps);
        error = nan(3,steps);
        for i = 1:steps-1
            xdot = (x(:,i+1) - x(:,i))/deltaT;                                  % Calculate velocity at discrete time step
            J = igus.model.jacob0(qMatrix(i,:));                                            % Get the Jacobian at the current state
            J = J(1:3,:);                                                           % Take only first 2 rows
            m(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
            if m(:,i) < minManipMeasure
                qdot = inv(J'*J + 0.01*eye(7))*J'*xdot;
            else
                qdot = pinv(J) * xdot;                                               % Solve velocitities via RMRC
            end

            error(:,i) = xdot - J*qdot;
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';                         % Update next joint state
        end
    end

    figure(1)
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
    % igus.model.plot(qMatrix,'trail','r-')                                               % Animate the robot

    for i = 1:steps
        igus.model.animate(qMatrix(i,:))
        pause(0.01)
    end




end