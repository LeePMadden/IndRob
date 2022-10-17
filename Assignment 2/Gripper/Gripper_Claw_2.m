%% Gripper File


classdef Gripper_Claw_2 < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end

    methods%% Class for UR5 robot simulation

function self = Gripper_Claw_2(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace
        
% robot = 
self.GetGripper();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetUR5Robot
% Given a name (optional), create and return a UR5 robot model
function GetGripper(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['Gripper',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    L1 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L2 = Link('d',0,'a',0.0001,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    
    self.model = SerialLink([L1 L2],'name',name);


end

%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)


    for linkIndex = 1:2
        %if self.useGripper && linkIndex == self.model.n
        %    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread('RobotiqGripperExperiment_1.ply'); %#ok<AGROW>
        %else

        if linkIndex == 2
             [ faceData, vertexData, plyData{linkIndex+1}] = plyread(['RobotiqGripperExperiment_2.ply'],'tri'); %#ok<AGROW>
        %end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;

        end

    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end

function close(self)

    self.model.teach([0,45*pi/180]);
end

    end
end 