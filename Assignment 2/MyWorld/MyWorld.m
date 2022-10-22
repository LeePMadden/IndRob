%% My World 

% Loads in static objects in the simulated environment 
function MyWorld 

PlaceObject('garagedoor.ply', [3,-3,0])
hold on

PlaceObject('compactor.ply', [0.25,-0.5,0])
hold on 

PlaceObject('table.ply', [0.35,0.45,0])
hold on

% PlaceObject('controlpanel.ply', [1.5,1.4,0.2])
% hold on
% PlaceObject('compactor.ply', [0,-0.4,0])

PlaceObject('glassbin.ply', [0.95,0.45,0])
hold on

    %% Safety Fence
    % https://sketchfab.com/3d-models/emergency-stop-button-012e4809a41445ca9de17286f677fabb
% PlaceObject('SafetyFence_1.ply', [0.75,-2.5,0])
% hold on

PlaceObject('SafetyFence_1.ply', [-1.3,-2.5,0])
hold on

PlaceObject('SafetyFence_1.ply', [-1.3,1.5,0])
hold on
    
fence = PlaceObject('SafetyFence_2.ply', [0.5,-1.8,0])
hold on

% PlaceObject('SafetyFence_2.ply', [-2,0.5,0])
% hold on






% PlaceObject('man.ply', [1.5,1.5,0])
% hold on


    %% Fire Extinguisher 
    % https://sketchfab.com/3d-models/fire-extinguisher-5676b179b3b744c0aaae53a3dcea2300
PlaceObject('FireExtinguisher.ply', [0.35, 1.5, 0.4]);

end

