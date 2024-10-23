classdef IGUSReBel < RobotBaseClass
    %% IGUS ReBel 6DOF Robot

    properties(Access = public)              
        plyFileNameStem = 'IGUSReBel';
    end
    
    methods
%% Define robot Function 
        function self = IGUSReBel(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
           
            self.PlotAndColourRobot(); 
            self.model.teach;
        end

%% Create the robot model
        function CreateModel(self)          
            link(1) = Link('d',0.252,'a',0,'alpha',-pi/2,'qlim',deg2rad([-179 179]));
            link(2) = Link('d',0,'a',0.2415,'alpha',-pi,'qlim',deg2rad([-80 140]),'offset',-pi/2);
            link(3) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-80 140]),'offset',-pi/2);
            link(4) = Link('d',0.3,'a',0,'alpha',pi/2,'qlim',deg2rad([-179 179]));
            link(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-95 95]));
            link(6) = Link('d',0.129,'a',0,'alpha',0,'qlim',deg2rad([-179 179]));
          
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end