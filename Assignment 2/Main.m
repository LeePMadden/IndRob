close all

clc

    %% Generate Environment 
    
    %% Workspace Size 
    % workspace = [-2 2 -2 2 -0.3 2];
    
    % workspace = [-10 10 -10 10 0 10];
    
    %% Environment Setup
    surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
    
    hold on; 
    
    MyWorld

     UR3Pose = [0.35,0.35,0.35];
    UR20Pose = [-1.2, 0.2, 0];

    
     UR3arm = UR3;
    UR20arm = UR20;


    UR20arm.gripper = true;
    
    % UR20arm = UR20;
    
    UR3arm.model.base = transl(UR3Pose) * trotz(pi);

    UR20arm.model.base = transl(UR20Pose) * trotz(pi);
    
    q = [0,0,0,0,0,0]; %zeros(1,6);


    
    UR3arm.model.animate(q)
    UR20arm.model.animate(q)
    
    %% Need to input cans and boxes
    
%     cPose = [0.75 0.2  0.3;  % Position 1
%              0.75 0.3  0.3; % Position 2
%              0.6  0.2  0.3;  % Position 3
%              0.7  0.3  0.3; % Position 4
%              0.6  0.1  0.3; % Position 5https://github.com/LeePMadden/IndRob/blob/main/Assignment%202/%40UR20/UR20gripper.ply
%              0.7  0.1  0.3; % Position 6
%              0.6  0.2  0.3; % Position 7
%              0.7  0.15 0.3; % Position 8
%              0.7  0.3  0.3]; % Position 9

cPose = [1 0.2  0.3; 
        1 0.2  0.3;
        1 0.2  0.3;
        1 0.2  0.3;
        1 0.2  0.3;
        1 0.2  0.3;
        1 0.2  0.3;
        1 0.2  0.3;
        1 0.2  0.3];
    
    [cCols, cRows] = size(cPose);
    
    Input = [0.3 0 0.55];
    
    PlaceObject('cancube.ply', [-0.5,-0.5,0.15])
    hold on

    PlaceObject('cancube.ply', [-2,-0,0.15])
    hold on
    PlaceObject('cancube.ply', [-2,-0.25,0.15])
    hold on
    PlaceObject('cancube.ply', [-2,-0.5,0.15])
    hold on
    PlaceObject('cancube.ply', [-2,-0.25,0.35])
    hold on
    PlaceObject('cancube.ply', [-2,-0.5,0.35])
    hold on


% retract(UR3arm)
% 
% 
% for i = 0:0.1:1.5
% 
%     delete(fence)
% 
% fence = PlaceObject('SafetyFence_2.ply', [1.5 + i,0,0])
% 
% pause(5)
% 
% end

% calculates and animates trajectory from current q values to cartesian end
% effector



%% Crushing Function ~ 
% function crush
        
%% Can Meshes 


resPose = [deg2rad(-7.2) deg2rad(-144) deg2rad(137) ...
    deg2rad(-203) deg2rad(-86.4) deg2rad(-36)];

resPose_2 = [deg2rad(-56.9) deg2rad(-79.2) deg2rad(82.6) ...
    deg2rad(-95) deg2rad(-90) deg2rad(123)];
    
    UR3arm.model.animate(resPose)

    UR20arm.model.animate(resPose_2)
% 
% 
% 
% block = [-0.5,-0.5,0.3]
% 
% carry = false;
% 
% trajectory_q2c(UR20arm, block, carry)
% 
% location = [-1.5 0.5 1.5];
% 
% % carry = true;
% % 
% % trajectory_q2c(UR20arm, block, carry)
% 
for j = 1:1

    disp(j)
    
    cTarget = cPose(j,:);

    carry = false;

    % call fucntion
    disp('picking up can')
    trajectory_q2c(UR3arm, cTarget, carry)
    
    UR3arm.model.getpos;

    carry = true;
    
    disp('dropping off can')
    trajectory_q2c(UR3arm, Input, carry)

end

retract(UR3arm)


% 
% end


function retract(robotarm)

% Desired pose in q 
resPose = [deg2rad(-7.2) deg2rad(-144) deg2rad(137) ...
    deg2rad(-203) deg2rad(-86.4) deg2rad(-36)];

% current pose in q
currPose = robotarm.model.getpos();

%plot trajectory 
traj = jtraj(currPose,resPose,40);

    % iterates through joint trajectories
    for k = 1:size(traj,1)
    
        hold on
    
        traj_1 = traj(k,:);
    
        robotarm.model.animate(traj);
    
        drawnow();
    
        % slow down speed 
        pause(5)
    end

end



% %% Crushing Function ~ 
% % function crush
%         
% %% Can Meshes 
% 
% for i = 1:9
% 
%     % create variable name based on iterator
%     cLoc(i) = PlaceObject('can.ply',cPose(i,:)); 
%     cMesh(i) = PlaceObject('can.ply',cPose(i,:));
% 
% end 
% 
% for j = 1:cCols
% 
%     disp(j)
%     
%     cTarget = cPose(j,:);
% 
%     q = UR3arm.model.getpos();
% 
%     UR3_CurrPose = UR3arm.model.fkine(UR3arm.model.getpos());
%     
%     cLoc = transl(cPose(1,1),cPose(1,2),cPose(1,3)) * trotx(pi);
%     
%     UR3_Ikcon = UR3arm.model.ikcon(cLoc,q);
%     
%     UR3_Traj = jtraj(q,UR3_Ikcon,25);
%     
%     %    iterates through joint trajectories
%     for k = 1:size(UR3_Traj,1)
%     
%         hold on
%     
%         traj = UR3_Traj(k,:);
%     
%         UR3arm.model.animate(traj);
%     
%         UR3_Fkine = UR3arm.model.fkine(UR3arm.model.getpos());
%     
%         drawnow();
% 
%     %        slow down speed 
%         pause(0.05)
%     end
%     
%     q = UR3arm.model.getpos();
%     
%     UR3_CurrPose = UR3arm.model.fkine(q);
%     
%     InputLoc = transl(Input(1,1),Input(1,2),Input(1,3)) * trotx(pi);
%     
%     UR3_Ikcon = UR3arm.model.ikcon(InputLoc,q);
%     
%     UR3_Traj = jtraj(q,UR3_Ikcon,25);
%     
%     %    iterates through joint trajectories
%     for k = 1:size(UR3_Traj,1)
%     
%         hold on
%     
%         traj = UR3_Traj(k,:);
%     
%         UR3arm.model.animate(traj);
%     
%         UR3_Fkine = UR3arm.model.fkine(UR3arm.model.getpos());
% 
%     
%         drawnow();
%     %        slow down speed 
%         pause(0.05)
%     end
% 
% end
% % 
% % end
% 
% 
% function retract(robotarm)
% 
% % Desired pose in q 
% retPose = [deg2rad(-7.2) deg2rad(-144) deg2rad(137) ...
%     deg2rad(-203) deg2rad(-86.4) deg2rad(-36)];
% 
% % current pose in q
% currPose = robotarm.model.getpos();
% 
% %plot trajectory 
% traj = jtraj(currPose,resPose,25);
% 
%     % iterates through joint trajectories
%     for k = 1:size(traj,1)
%     
%         hold on
%     
%         traj_1 = traj(k,:);
%     
%         robotarm.model.animate(traj);
%     
%         drawnow();
%     
%         % slow down speed 
%         pause(0.05)
%     end
%             
% end

function trajectory_q2c(robotarm, nextPosition, holding)

rehash path
        
        nextLoc = transl(nextPosition(1,1),nextPosition(1,2),nextPosition(1,3)) * trotx(pi);

        robotPos = robotarm.model.getpos();

        ikcon = robotarm.model.ikcon(nextLoc, robotPos);

        traj = jtraj(robotPos, ikcon,25);
        
        %iterates through joint trajectories 
        for k = 1:size(traj,1)
        
            hold on
        
            traj_1 = traj(k,:);
        
            robotarm.model.animate(traj_1);

            robotPos = robotarm.model.getpos();

            robotLoc = robotarm.model.fkine(robotPos);

            EELoc = robotLoc(1:3,4)' - [0,0,0.1];

            %slow down speed 

            if holding == true
               
                load = PlaceObject('can.ply',EELoc);
%                load = PlaceObject('cancube.ply',EELoc);
               
               drawnow();

%                pause(0.01)

               delete(load);

            else 

                drawnow();
                pause(0.1)

            end
      
        end

end

function getMeMyBeer(position)

    PlaceObject('can.ply',position);

end