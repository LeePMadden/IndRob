classdef UR20 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-2 2 -2 2 -0.3 2];   
         
    end
    
    methods%% Class for UR20 robot simulation
function self = UR20(toolModelAndTCPFilenames)
    if 0 < nargin
        if length(toolModelAndTCPFilenames) ~= 2
            error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
        end
        self.toolModelFilename = toolModelAndTCPFilenames{1};
        self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
    end
    
    self.GetUR20Robot();
    self.PlotAndColourRobot();%robot,workspace);

    drawnow
end
%% GetUR20Robot
% Given a name (optional), create and return a UR20 robot model
function GetUR20Robot(self)
            pause(0.001);
            name = ['UR_20_',datestr(now,'yyyymmddTHHMMSSFFF')];

        L1 = Link('d',0.3,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        L2 = Link('d',0.27,'a',0.65,'alpha',pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
        L3 = Link('d',0.24,'a',0.62,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
        L4 = Link('d',0.22,'a',0,'alpha',-pi/2,'offset',pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
        L5 = Link('d',0.21,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        L6 = Link('d',0.1,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UR20Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
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
    end
end