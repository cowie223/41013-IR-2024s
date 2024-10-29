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

% Setup points for default movement. Later these will be replaced with GUI
% inputs.

igusUseLoc = [-0.75,0,1];
igusUsePose = rotx(pi)*rotz(-pi/2);
igusUseTransf = SE3(igusUsePose,igusUseLoc);

igusChargeLoc = [-1.2,-0.9,0.85];
igusChargePose = rotx(-pi/2)*rotz(0);
igusChargeTransf = SE3(igusChargePose,igusChargeLoc);

aboveUseLoc = [-0.75,0,1.2];
aboveUseTransf = SE3(igusUsePose,aboveUseLoc);

aboveChargeLoc = [-1.2,-0.75,1.2];
aboveChargeTransf = SE3(igusChargePose,aboveChargeLoc);

dobotChargeLoc = [-1.11,-1,0.75];
dobotChargePose = rotx(pi/2)*rotz(0);
dobotChargeTransf = SE3(dobotChargePose,dobotChargeLoc);

belowChargeLoc = [-1.11,-1,0.7];
belowChargeTransf = SE3(dobotChargePose,belowChargeLoc);

steps = 100;

igusSteps = [aboveUseTransf igusUseTransf aboveUseTransf aboveChargeTransf igusChargeTransf];
dobotSteps = [belowChargeTransf dobotChargeTransf];

%% Move to point 'Use' & 'Charge' --> IGUS only

q0 = q0igus;
[~,cols] = size(igusSteps);

for i=1:1:cols

    q1 = igus.model.ikcon(igusSteps(1,i),q0);                          % Determine required joint angles via Inverse Kinematics
    s = lspb(0,1,steps);                                        % Trapezoidal trajectory generation
    qMatrix = nan(steps,7);

    for j = 1:steps                                             
        qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
    end
    
    for j=1:1:steps                                             % Animate movement through trajectory           
        newQ=qMatrix(j,:);
        igus.model.animate(newQ);
        pause(0.01);
    end
    
    pause(0.2);
    
    q0 = q1;
end

q0 = q0dobot;
[~,cols] = size(dobotSteps);

for i=1:1:cols

    q1 = dobot.model.ikcon(dobotSteps(1,i),q0);                          % Determine required joint angles via Inverse Kinematics
    s = lspb(0,1,steps);                                        % Trapezoidal trajectory generation
    qMatrix = nan(steps,5);

    for j = 1:steps                                             
        qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
    end
    
    for j=1:1:steps                                             % Animate movement through trajectory           
        newQ=qMatrix(j,:);
        dobot.model.animate(newQ);
        pause(0.01);
    end
    
    pause(0.2);
    
    q0 = q1;
end

%% New section

dobot.model.animate(q0dobot);
igus.model.animate(q0igus);