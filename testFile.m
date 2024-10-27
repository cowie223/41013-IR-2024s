% % Load the PLY file and create a patch object for the tool
% tool = pcread('PhoneHolder.ply');  

axis([-2,2,-2,2,0,2.5]);
hold on
PlaceObject('completeEnvironment.ply',[0,0,0]);

igusLoc = transl([-1.4,-1.35,1.03]);
igus = IGUSReBel(igusLoc);

dobotLoc = transl([-1.4,-1.15,0.55]);
dobot = DobotMagician(dobotLoc);

q0igus = igus.model.getpos;
q0dobot = dobot.model.getpos;

% % Get the end-effector pose as a 4x4 transformation matrix
% endEffectorPose = igus.model.fkine(igus.model.getpos).T;
% endEffectorPosition = endEffectorPose(1:3, 4)
% 
% PlaceObject('PhoneHolder.ply', endEffectorPosition); % Place tool at the end effector

%% Move to point 'Use'

q0 = q0igus;
steps = 100;
R = rotx(pi)*rotz(0);                                       % Determine RPY target values   
T = [-0.75,0,1];                                         % Define translation target
Transf = SE3(R,T);                                          % Define transformation matrix
q1 = igus.model.ikcon(Transf, q0);                    % Determine required joint angles via Inverse Kinematics
s = lspb(0,1,steps);                                        % Trapezoidal trajectory generation
qMatrix = nan(steps,7);
for j = 1:steps                                             
    qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
end

for j=1:1:steps                                             % Animate movement through trajectory, whilst updating the gripper location           
    newQ=qMatrix(j,:);
    igus.model.animate(newQ);
    pause(0.01);
end

q0 = q1;

R = rotx(-pi/2)*rotz(0);                                       % Determine RPY target values   
T = [-1,-0.9,0.85];                                         % Define translation target
Transf = SE3(R,T);                                          % Define transformation matrix
q1 = igus.model.ikcon(Transf, q0);                    % Determine required joint angles via Inverse Kinematics
s = lspb(0,1,steps);                                        % Trapezoidal trajectory generation
qMatrix = nan(steps,7);
for j = 1:steps                                             
    qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
end

for j=1:1:steps                                             % Animate movement through trajectory, whilst updating the gripper location           
    newQ=qMatrix(j,:);
    igus.model.animate(newQ);
    pause(0.01);
end

                