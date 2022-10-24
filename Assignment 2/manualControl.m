function [ ] = manualControl(app, robot)
% slider used
% value from 0-100 added to workspace
% turn 0-100 into a number in the joint limit
% animate
%% Environment Setup
clf
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
app.manualControlOn = true;

%% manual control section
app.jointxyzmovementButton.Value = 0;
while app.manualControlOn
    pause(0.2)
    emergencyStop(app)
    if robot == 1 %UR3
        
        if ~app.ManualControlButton.Value
            app.manualControlOn = false;
        end
        
        %enable xyz movement
        if app.jointxyzmovementButton.Value
            vx = app.x;
            vy = app.y;
            vz = app.z;
            
            dx = [vx;vy;vz;0;0;0];
            
            lambda = 0.5;
            J = UR3arm.model.jacob0(q);
            Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
            dq = Jinv_dls*dx;
            
            q = q + dq'*0.15;
            
            UR3arm.model.animate(q);
            app.x = 0;
            app.y = 0;
            app.z = 0;
        else %joint movement
            q = [deg2rad(app.q1Slider.Value) deg2rad(app.q2Slider.Value) deg2rad(app.q3Slider.Value)...
                deg2rad(app.q4Slider.Value) deg2rad(app.q5Slider.Value) deg2rad(app.q6Slider.Value)];
            
            oldQ = UR3arm.model.getpos();
            
            UR3Trajectory = jtraj(oldQ,q,20);
            
            %animate movement of robot arm
            for j = 1:size(UR3Trajectory,1)
                trajectory = UR3Trajectory(j,:);
                UR3arm.model.animate(trajectory);
                drawnow();
                pause(0.05)
            end
        end
        
        
        
    end
    if robot == 2 %UR20
        if ~app.ManualControlButton_2.Value
            app.manualControlOn = false;
        end
        
        if app.jointxyzmovementButton.Value
            vx = app.x2;
            vy = app.y2;
            vz = app.z2;
            
            dx = [vx;vy;vz;0;0;0];  
            
            lambda = 0.5;
            J = UR20arm.model.jacob0(q);
            Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
            dq = Jinv_dls*dx;
            
            q = q + dq'*0.15;
            
            UR20arm.model.animate(q);
            app.x2 = 0;
            app.y2 = 0;
            app.z2 = 0;
            
        else
            
            q = [deg2rad(app.q1Slider_2.Value) deg2rad(app.q2Slider_2.Value) deg2rad(app.q3Slider_2.Value)...
                deg2rad(app.q4Slider_2.Value) deg2rad(app.q5Slider_2.Value) deg2rad(app.q6Slider_2.Value)];
            
            oldQ = UR20arm.model.getpos();
            
            UR3Trajectory = jtraj(oldQ,q,20);
            
            %animate movement of robot arm
            for j = 1:size(UR3Trajectory,1)
                trajectory = UR3Trajectory(j,:);
                UR20arm.model.animate(trajectory);
                drawnow();
                pause(0.05)
            end
        end
    end
end


end
