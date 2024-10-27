% % Load the PLY file and create a patch object for the tool
% tool = pcread('PhoneHolder.ply');  

axis([-2,2,-2,2,0,2.5]);
hold on
PlaceObject('completeEnvironment.ply',[0,0,0]);

igusLoc = transl([-1.4,0.55,1.03]);
igus = IGUSReBel(igusLoc);

dobotLoc = transl([-1.4,-1.15,0.55]);
dobot = DobotMagician(dobotLoc);

q0igus = igus.model.getpos
q0dobot = dobot.model.getpos;

% % Get the end-effector pose as a 4x4 transformation matrix
% endEffectorPose = igus.model.fkine(igus.model.getpos).T;
% endEffectorPosition = endEffectorPose(1:3, 4)
% 
% PlaceObject('PhoneHolder.ply', endEffectorPosition); % Place tool at the end effector

%% Move to point 'Use'

q1igus = [-0.75,0,1];
q2igus = [-1,-0.75,0.75];

RMRC3(q0igus,q1igus,q2igus,igus);