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


    %UR20arm.gripper = true;
    
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

        
%% Can Meshes 

resPose = [deg2rad(-7.2) deg2rad(-144) deg2rad(137) ...
    deg2rad(-203) deg2rad(-86.4) deg2rad(-36)];

resPose_2 = [deg2rad(-56.9) deg2rad(-79.2) deg2rad(82.6) ...
    deg2rad(-95) deg2rad(-90) deg2rad(123)];
    
    UR3arm.model.animate(resPose)

    UR20arm.model.animate(resPose_2)

for j = 1:4

    if j < 4

        disp(j)
        
        cTarget = cPose(j,:);
    
        cancarry = false;
    
        cubecarry = false;
    
        % call fucntion
        disp('picking up can')
        
        %trajectory_q2c(UR3arm, cTarget, cancarry, UR20arm,  [-0.5,-0.5,0.15], cubecarry)
        Lab9Solution_Question1(UR3arm, cTarget) % RMRC Attempt

        UR3arm.model.getpos;
    
        cancarry = true;
        
        disp('dropping off can')
        %trajectory_q2c(UR3arm, Input, cancarry, UR20arm,  [-0.5,-0.5,0.15], cubecarry)
        Lab9Solution_Question1(UR3arm, Input)

    else

        disp(j)
        
        cTarget = cPose(j,:);
    
        cancarry = false;
    
        cubecarry = false;
    
        % call fucntion
        disp('picking up cube')
        %trajectory_q2c(UR3arm, cTarget, cancarry, UR20arm,  [-0.5,-0.5,0.15], cubecarry)
        Lab9Solution_Question1(UR20arm, cTarget)

        UR3arm.model.getpos;
    
        cubecarry = true;
        
        disp('dropping off cube')
        %trajectory_q2c(UR3arm, Input, cancarry, UR20arm,  [0,-2,0.4], cubecarry)
        Lab9Solution_Question1(UR20arm, Input)

    end

end

retract(UR3arm)

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

function getMeMyBeer(position)

    PlaceObject('can.ply',position);

end


%% RMRC Attempt
function Lab9Solution_Question1(robot,target)
% 1.1) Set parameters for the simulation
t = 3;             % Total time (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.4;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint anglesR
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error
% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);            % Trapezoidal trajectory scalar
origin = robot.model.fkine(robot.model.getpos)
 for i=1:steps
     x(1,i) = (1-s(i))*origin(1) + s(i)*target(1); % Points in x
     x(2,i) = (1-s(i))*origin(2) + s(i)*target(2); % Points in y
     x(3,i) = (1-s(i))*origin(3) + s(i)*target(3); % Points in z
     theta(1,i) = 0;            % Roll angle 
     theta(2,i) = pi;           % Pitch angle
     theta(3,i) = 0;            % Yaw angle
 end


T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];         % Create transformation of first point and angle
q0 = robot.model.getpos;                                                           % Initial guess for joint angles
qMatrix(1,:) = robot.model.ikcon(T,q0);                                          % Solve joint angles to achieve first waypoint
% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:))                                    % Get forward transformation at current joint state
                                                                           % THIS LINE IS THE ROOT OF OUR PROBLEMS
                                                                           % FORCING IT TO BE REAL CREATES EVEN MORE ERRORS

    deltaX = x(:,i+1) - T(1:3,4);                                          % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                    % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                       % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                           % Calculate rotation matrix error
    S = Rdot*Ra';                                                          % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                             % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                           % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                           % Calculate end-effector velocity to reach next waypoint.
    J = robot.model.jacob0(qMatrix(i,:));                                         % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                                      % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                  % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                              % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                            % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                % If next joint angle is lower than joint limit...
            qdot(i,j) = 0;                                                 % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)            % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0;                                                 % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                      % Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                              % For plotting
    angleError(:,i) = deltaTheta;                                          % For plotting
end
    for i = 1:steps
        robot.model.animate(qMatrix(i,:));
        pause(0.05)
    end
end