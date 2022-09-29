close all

clc

%% Generate Environment 

%% Workspace Size 
% workspace = [-2 2 -2 2 -0.3 2];

%%workspace = [-10 10 -10 10 0 10];


%% Environment Setup
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

hold on; 

UR3Pose = [0.3,0.2,0.62];

UR3_M = UR3Main;

UR3_M.model.base = transl(UR3Pose) * trotz(pi);

UR3_M_CurrPose = UR3_M.model.fkine(UR3_M.model.getpos());

q = [0,0,0,0,0,0]; %zeros(1,6);

PlaceObject('conveyerthin.ply', [0.75,0.75,0])

PlaceObject('conveyer.ply', [-1,1,0])

PlaceObject('workshoptable.ply', [0,0.2,0.6])

PlaceObject('controlpanel.ply', [0,1,0.2])

% PlaceObject('compactor.ply', [0,-0.4,0])

PlaceObject('glassbin.ply', [0.8,-0.3,0])

    %% Safety Fence
    % https://sketchfab.com/3d-models/emergency-stop-button-012e4809a41445ca9de17286f677fabb
PlaceObject('SafetyFence_1.ply', [0.75,-1.8,0])
hold on

PlaceObject('SafetyFence_1.ply', [-1.3,-1.8,0])
hold on
    
PlaceObject('SafetyFence_2.ply', [-2,0,0])
hold on

PlaceObject('SafetyFence_2.ply', [1.5,0,0])
hold on

    %% Fire Extinguisher 
    % https://sketchfab.com/3d-models/fire-extinguisher-5676b179b3b744c0aaae53a3dcea2300
PlaceObject('FireExtinguisher.ply', [0.35, 1.5, 0.4]);

UR3_M.model.animate(q)

%% Need to input cans and boxes
