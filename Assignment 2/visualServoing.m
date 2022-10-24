close all
clf
%% visual servoing

% sign held up by one robot ~ avoided by the other 
% the ur20 will hold up points and the ur3 will retreat from it

%Notes: not entirely working - velocity matrix v is 

%% Environment Setup
% create workspace with concrete floor
surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

hold on;


% call my world to load in static objects
MyWorld

% robot poses
UR3Pose = [0.5,0.35,0.35];
UR20Pose = [-1.2, 0.2, 0];

% assign instances of robots
UR3arm = UR3;
UR20arm = UR20;

% assign robot model to robot for simplicity
robot = UR3arm.model;

% adjust base location of robots
% UR3arm.model.base = transl(UR3Pose) * trotz(pi);
robot.base = transl(UR3Pose) * trotz(pi);
UR20arm.model.base = transl(UR20Pose) * trotz(pi);

% resting pose of joint angles (obtained from teach ~ just picked for
% aestethics)
resPose = [deg2rad(-5.6) deg2rad(-72) deg2rad(-35.8) ...   % initial pose for UR3
    deg2rad(-44.6) deg2rad(93.8) deg2rad(-101)];

q0 = resPose;

resPose_2 = [deg2rad(0) deg2rad(-90) deg2rad(89.4) ...    % initial pose for UR20
    deg2rad(-174) deg2rad(-97.2) deg2rad(274)];

% resPose_3 = [deg2rad(0) deg2rad(-79.2) deg2rad(75.8) ...    % next pose for UR20
%     deg2rad(-174) deg2rad(-90) deg2rad(274)];

% animate resting poses
% UR3arm.model.animate(resPose);
UR20arm.model.animate(resPose_2);

% UR20 end effector position to position 3d points
EE = UR20arm.model.fkine(UR20arm.model.getpos);

% Adding camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR3camera');
% frame rate
fps = 25;

% Create image target (points in the image plane) 
pStar = [500; 500];

%Create 3D points
P=[EE(1,4)+0.5;
    EE(2,4);
    EE(3,4)];

plot_sphere(P, 0.1, 'b')
lighting gouraud
light


%Define values
% gain of the controler
lambda = 0.6;
% depth of the IBVS
depth = mean (P(1,:));
% epsilon for damping
epsilon = 0.025;


robot.animate(q0)
%Display UR3
Tc0= robot.fkine(q0);

drawnow

% plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0,'scale',0.005); % plot camera
plot_sphere(P, 0.05, 'b')                          % plot sphere

lighting gouraud
light

% Project points to the image:
p = cam.plot(P, 'Tcam', Tc0);

% Camera view
cam.clf();  % clear camera 
cam.plot(pStar, '*'); % plot asterisk at target
cam.hold(true);      
cam.plot(P, 'Tcam', Tc0, 'o');

pause(0.5);     % pause to let everything plot or it goes through too quickly!
cam.hold(true);
cam.plot(P);

pause(0.5);     %pause again because it takes agesss

%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];

% plot trapezoidal velocity
% steps = 20;
% s = lspb(0,1,steps);
% qMatrix = nan(steps,6);
% for i = 1:steps
% qMatrix(i,:) = (1-s(i))*resPose_2 + s(i)*resPose_3;
% end

% % movement of UR20 forward
% for i = 1:size(qMatrix,1)
% UR20arm.model.animate(qMatrix(i,:))
% pause(1)
% end


ksteps = 0;

while true

    ksteps = ksteps + 1;

        % compute the view of the camera
    uv = cam.plot(P);

    % compute image plane error as a column
    e = pStar-uv;   % feature error
    e = e(:);
    Zest = [];
    
    % compute the Jacobian
    if isempty(depth)
        % exact depth from simulation (not possible in practice)
        pt = homtrans(inv(Tcam), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else
        J = cam.visjac_p(uv, depth );
    end

        % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
%     fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

      currPose = robot.getpos();        %get current pose
      J = cam.visjac_p(uv, depth);      %compute jacobian

     % compute robot's Jacobian and inverse
    J2 = robot.jacobn(currPose);         % current joing jacobian


    n(ksteps) = sqrt(det(J2*J2'));      % meaure of manipubility 

    % Applying DLS
        if n(ksteps) < epsilon                          % If manipulability is less than given threshold
            damp = (1 - n(ksteps)/epsilon)*5E-1;     % Damped least square
        else
            damp = 0;                                % Otherwise no need to dampen
        end

    Jinv =  pinv(J2);
    % get joint velocities



    qp = Jinv*v(1,:,1);

     %Maximum angular velocity cannot exceed 180 degrees/s
     ind=find(qp>pi);
     if ~isempty(ind)
         qp(ind)=pi;
     end
     ind=find(qp<-pi);
     if ~isempty(ind)
         qp(ind)=-pi;
     end

        %Update joints 
        q = q0 + (1/fps)*qp;

    % plot trapezoidal velocity
    % steps = 20;
    % s = lspb(0,1,steps);
    % qMatrix = nan(steps,6);
    % for i = 1:steps
    % qMatrix(i,:) = (1-s(i))*resPose_2 + s(i)*resPose_3;
    % end

        % for i = 1:size(qMatrix,1)
    % UR20arm.model.animate(qMatrix(i,:))
    % pause(1)
    % end
        robot.animate(q');

        disp(q)

        %Get camera location
        Tc = robot.fkine(q);


        cam.T = Tc(:,:,6);

        drawnow

        % Update the history variables:
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist]; %for plotting
        currPose = robot.getpos();
        disp(currPose);
        pause(1/fps)

        if ~isempty(200) && (ksteps > 100)
            break;
        end
%         
        q0 = q;             % Update current joint position
end



