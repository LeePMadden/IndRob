%% Past Code

% compilation of previous coding functions that are now reduntant due to
% the implementation of new functions
%
% The code kept here is just for tracking

%% Crushing Function ~ 

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


%% Trajectories using jtraj

% redundant due to the implementation of RMRC

function trajectory_q2c(UR3robotarm, UR3nextPosition, UR3holding, UR20robotarm, UR20nextPosition, UR20holding)

rehash path
        
    UR3nextLoc = transl(UR3nextPosition(1,1),UR3nextPosition(1,2),UR3nextPosition(1,3)) * trotx(pi);
    UR3robotPos = UR3robotarm.model.getpos();
    UR3ikcon = UR3robotarm.model.ikcon(UR3nextLoc, UR3robotPos);
    UR3traj = jtraj(UR3robotPos, UR3ikcon,25);

    UR20nextLoc = transl(UR20nextPosition(1,1),UR20nextPosition(1,2),UR20nextPosition(1,3)) * trotx(pi);
    UR20robotPos = UR20robotarm.model.getpos();
    UR20ikcon = UR20robotarm.model.ikcon(UR20nextLoc, UR3robotPos);
    UR20traj = jtraj(UR20robotPos, UR20ikcon,25);
    
    
    %iterates through joint trajectories 
    for k = 1:size(UR3traj,1)
    
        hold on
    
        UR3traj_1 = UR3traj(k,:);

        UR20traj_1 = UR20traj(k,:);
    
        UR3robotarm.model.animate(UR3traj_1);
        UR3robotPos = UR3robotarm.model.getpos();
        UR3robotLoc = UR3robotarm.model.fkine(UR3robotPos);
        UR3EELoc = UR3robotLoc(1:3,4)' - [0,0,0.2];


        UR20robotarm.model.animate(UR20traj_1);
        UR20robotPos = UR20robotarm.model.getpos();
        UR20robotLoc = UR20robotarm.model.fkine(UR20robotPos);
        UR20EELoc = UR20robotLoc(1:3,4)' - [0,0,0.5];

        %slow down speed 
        if UR3holding == true
            load(1) = PlaceObject('can.ply',UR3EELoc);
        else
            pause(0.05)
        end
        if UR20holding == true
            load(2) = PlaceObject('canCUBE.ply',UR20EELoc);  
        else
            pause(0.05)    
        end
        drawnow();
        if (UR3holding || UR20holding)
        delete(load(:)) % destroy any cans or cubes we just created
        end
    end

end