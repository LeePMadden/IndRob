% 
% function collisionAvoidanceUR20
%% Environment Setup
% create workspace with concrete floor
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

hold on;

% call my world to load in static objects
MyWorld

%     PlaceObject('cancube.ply', [-0.5,-0.5,0.15])
%     hold on
% 
%     PlaceObject('cancube.ply', [-2,-0,0.15])
%     hold on
%     PlaceObject('cancube.ply', [-2,-0.25,0.15])
%     hold on
%     PlaceObject('cancube.ply', [-2,-0.5,0.15])
%     hold on
%     PlaceObject('cancube.ply', [-2,-0.25,0.35])
%     hold on
%     PlaceObject('cancube.ply', [-2,-0.5,0.35])
%     hold on


% robot positions
UR3Pose = [0.35,0.35,0.35];
UR20Pose = [-1.2, 0.2, 0];

% assign instances of robots
UR3arm = UR3;
UR20arm = UR20;


robot = UR20arm.model; % assign robot model to robot for simplicity (instead of changing all values in IsCollision function)

%UR20 q values ~ desired starting position
%UR20arm.gripper = true;
% UR20arm = UR20;
resPoseUR20_1 = [deg2rad(-56.9) deg2rad(-79.2) deg2rad(82.6) ...
    deg2rad(-95) deg2rad(-90) deg2rad(123)];            %resting locationn of UR20



% adjust base location of robots
% UR3arm.model.base = transl(UR3Pose) * trotz(pi);

robot.base = transl(UR20Pose) * trotz(pi);
UR3arm.model.base = transl(UR3Pose) * trotz(pi);


% animate resting poses
% UR3arm.model.animate(resPose);
resPose = deg2rad([0 -28.8 40 -80 -93.6 0]);
robot.animate(resPoseUR20_1);
UR3arm.model.animate(resPose);


% ending pose of joint angles (obtained from teach ~ just picked for
% aestethics)

seqpose1 = deg2rad([-56.9 -130 89.8 -51.8 -90 123]);

seqpose2 = deg2rad([-143 -130 89.8 -51.8 -90 123]);

seqpose3 = deg2rad([-158 -36 25 -80.6 -90 109]);


endingq = deg2rad([-163 -43.2 39.4 -87.8 -90 289]); 
    

resPose = deg2rad([0 -28.8 40 -80 -93.6 0]); % resting pose for UR3

% positions to travel to
pose1 = [1,0.2,0.3];
pose2 = [0.35,0.04,0.6];

% calculate required q values 

% q values sent to the ffunction
q1 = robot.getpos;
endpos = robot.fkine(endingq);
q2 = endingq;% UR3arm.model.ikcon(endpos);


% HIT BOX FOR PERSON'S SAFESPACE
 
centerpnt = [-0.6,-1.2,0.8];                                                                             % center off hitbox
side_1 = 0.8;                                                                                           %length of sides of hitbox
side_2 = 0.8; 
plotOptions.plotFaces = false;                                                                        % to display hitbox - make true
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side_2/2, centerpnt+side_1/2,plotOptions); % obtain vertices, faces and fac
axis equal

% Persons location

PlaceObject('man1.ply',[-0.6,-1.2,0]) % loads in the representation of an obstacle
hold on


% % HIT BOX FOR TABLE
% centerpnt2 = [0.35,0.45,0.15];                                                                           % center off hitbox
% side_3 = 0.5;                                                                                            %length of sides of hitbox
% side_4 = 0.5;
% plotOptions.plotFaces = false;                                                                           % to display hitbox - make true
% [vertex2,faces2,faceNormals2] = RectangularPrism(centerpnt2-side_4/2, centerpnt2+side_3/2,plotOptions);  % obtain vertices, faces and fac
% axis equal

% vertex = [vertex1; vertex2];                        % assign vertices of each obstacle into the one matrix
% faces = [faces1; faces2];                           % assign faces of each obstacle into the one matrix
% faceNormals = [faceNormals1; faceNormals2];         % assign the facenormals of each obstacle into the one matrix


pause(1)

qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];

pause(5)

% Check if pose is in collision
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            robot.animate(qMatrixJoin);
            size(qMatrix)
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,6) - 1) * pi;
            while IsCollision(robot,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,6) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end

disp(qMatrix)
% 
% pause(5)



% steps = 2;
% 
% qMatrix = nan(steps,6);                                             % Create memory allocation for variables
% 
% q1 = robot.model.getpos;
% 
% for i = 1:steps
%     qMatrix(i,:) = (1-s(i))*seqpose1 + s(i)*seqpose2;                   	% Generate interpolated joint angles
% end
% 
% robot.animate(qMatrix)



for i = 1:size(qMatrix,1)

robot.animate(qMatrix(i,:))
pause(0.1)

end
% robot.animate(qMatrix)

% 
% end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                disp('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end

