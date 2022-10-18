close all

clc

%% Generate Environment 

%% Workspace Size 
% workspace = [-2 2 -2 2 -0.3 2];

% workspace = [-10 10 -10 10 0 10];

%% Environment Setup
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

hold on; 

UR3Pose = [0.35,0.2,0.62];
% UR20Pose = [0.35,0.2,0.62];

UR3arm = UR3Main;

% UR20arm = UR20;

UR3arm.model.base = transl(UR3Pose) * trotz(pi);

q = [0,0,0,0,0,0]; %zeros(1,6);

PlaceObject('compactor.ply', [0.2,-1.25,0])
hold on 

PlaceObject('controlpanel.ply', [1.5,1.4,0.2])
hold on
% PlaceObject('compactor.ply', [0,-0.4,0])

PlaceObject('glassbin.ply', [0.8,0.2,0])
hold on

    %% Safety Fence
    % https://sketchfab.com/3d-models/emergency-stop-button-012e4809a41445ca9de17286f677fabb
PlaceObject('SafetyFence_1.ply', [0.75,-2.5,0])
hold on

PlaceObject('SafetyFence_1.ply', [-1.3,-2.5,0])
hold on
    
PlaceObject('SafetyFence_2.ply', [-2,0,0])
hold on

PlaceObject('SafetyFence_2.ply', [1.5,0,0])
hold on

    %% Fire Extinguisher 
    % https://sketchfab.com/3d-models/fire-extinguisher-5676b179b3b744c0aaae53a3dcea2300
PlaceObject('FireExtinguisher.ply', [0.35, 1.5, 0.4]);

UR3arm.model.animate(q)

%% Need to input cans and boxes

cPose = [0.75 0.2  0.3;  % Position 1
         0.75 0.3  0.3; % Position 2
         0.6  0.2  0.3;  % Position 3
         0.7  0.3  0.3; % Position 4
         0.6  0.1  0.3; % Position 5
         0.7  0.1  0.3; % Position 6
         0.6  0.2  0.3; % Position 7
         0.7  0.15 0.3; % Position 8
         0.7  0.3  0.3]; % Position 9

[cCols, cRows] = size(cPose);

Input = [0.2 -0.6 0.75];

    %% Can Meshes 
for i = 1:9
    % create variable name based on iterator
    cLoc(i) = PlaceObject('can.ply',cPose(i,:)); 
    % cMesh(i) = PlaceObject('can.ply',cPose(i,:));
end 

for j = 1:cCols

    disp(j)
    
    cTarget = cPose(j,:);

    q = UR3arm.model.getpos();

    UR3_CurrPose = UR3arm.model.fkine(UR3arm.model.getpos());
    
    cLoc = transl(cPose(1,1),cPose(1,2),cPose(1,3)) * trotx(pi);
    
    UR3_Ikcon = UR3arm.model.ikcon(cLoc,q);
    
    UR3_Traj = jtraj(q,UR3_Ikcon,25);
    
    % iterates through joint trajectories
    for k = 1:size(UR3_Traj,1)
    
        hold on
    
        traj = UR3_Traj(k,:);
    
        UR3arm.model.animate(traj);
    
        UR3_Fkine = UR3arm.model.fkine(UR3arm.model.getpos());
    
        %%%%% "Gripper"
    %                  cowHerd.cow{1}.base = UR3Fkine;
    %                  cowHerd.cow{1}.animate(0);
    
        drawnow();
        % slow down speed 
        pause(0.05)
    end
    
    q = UR3arm.model.getpos();
    
    UR3_CurrPose = UR3arm.model.fkine(q);
    
    InputLoc = transl(Input(1,1),Input(1,2),Input(1,3)) * trotx(pi);
    
    UR3_Ikcon = UR3arm.model.ikcon(InputLoc,q);
    
    UR3_Traj = jtraj(q,UR3_Ikcon,25);
    
    % iterates through joint trajectories
    for k = 1:size(UR3_Traj,1)
    
        hold on
    
        traj = UR3_Traj(k,:);
    
        UR3arm.model.animate(traj);
    
        UR3_Fkine = UR3arm.model.fkine(UR3arm.model.getpos());
    
        %%%%% "Gripper"
    %                  cowHerd.cow{1}.base = UR3Fkine;
    %                  cowHerd.cow{1}.animate(0);
    
        drawnow();
        % slow down speed 
        pause(0.05)
    end

end
