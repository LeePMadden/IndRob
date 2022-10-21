%% My World 

function MyWorld 

PlaceObject('garagedoor.ply', [0.5,-2.8,0])
hold on 

PlaceObject('compactor.ply', [0.25,-1.25,0])
hold on 

% PlaceObject('controlpanel.ply', [1.5,1.4,0.2])
% hold on
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


PlaceObject('man.ply', [1.5,1.5,0])
hold on


    %% Fire Extinguisher 
    % https://sketchfab.com/3d-models/fire-extinguisher-5676b179b3b744c0aaae53a3dcea2300
PlaceObject('FireExtinguisher.ply', [0.35, 1.5, 0.4]);

end

