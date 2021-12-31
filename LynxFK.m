
% The following code simulates the forward kinematics of a 5DOF Lynx robot
% serial manipulator. Figure 2 shows its movement from start to end
% position, Figure 1 shows the location of its end effector at points of
% its trajectory and Figure 3 shows the maximum potential workspace of the
% arm's end effector.

clear all
close all
clc
disp('The following code simulates the forward kinematics of a Lynx robot')

%% A series of joint angles
% The following variables are defined in the form of column-vectors with
% 4 rows each. Each row represents a different position (angle) of the joint.
% e.g. inititally we hae 60 degrees for q1 and -30 for q2.


%% Links Lengths
% syms l1 l2 l3 l4 l5;
l1 = 1;
l2 = 1;
l3 = 1;
l4 = 0;
l5 = 1;
%% Joint angles
syms q1 q2 q3 q4 q5;


%% distal table
% T_ij = eye(4);

%% Tip position
% These equations are derived from the Forward Kinematic model of the lynx
% robot
% robot

a = [0 l2 l3 0 0]';
alpha = [90 0 0 -90 0]';
d = [l1 0 0 0 l5]';
theta = [q1 q2 q3 q4 q5]';
T_ij1 = Transformation(a(1), alpha(1), d(1), theta(1))
T_ij2 = Transformation(a(2), alpha(2), d(2), theta(2))
T_ij3 = Transformation(a(3), alpha(3), d(3), theta(3))
T_ij4 = Transformation(a(4), alpha(4), d(4), theta(4))
T_ij5 = Transformation(a(5), alpha(5), d(5), theta(5))

% for i = 1:5
%     T_ij = T_ij * Transformation(a(i), alpha(i), d(i), theta(i))
% end

T_ij  = T_ij1 * T_ij2 * T_ij3 * T_ij4 * T_ij5;

% xt = T_ij(1, 4)
% yt = T_ij(2, 4)
% zt = T_ij(3, 4)

xt = cosd(q1)*((l5*cosd(q2+q3+q4))+(l3*cosd(q2+q3))+(l2*cosd(q2)))
yt = sind(q1)*((l5*cosd(q2+q3+q4))+(l3*cosd(q2+q3))+(l2*cosd(q2)))
zt = (l5*sind(q2+q3+q4))+(l3*sind(q2+q3))+(l2*sind(q2))+l1

%% Plot the workspace of the robot
stepq1 = 30; 
step = 30;
points = 180/stepq1;
figure (2)
set(2,'position',[1243 190 560 420])

xwork = zeros(points^2) ; % reserving space for the variables, because
ywork = zeros(points^2) ; % otherwise they would be created later within a loop.
zwork = zeros(points^2) ;
QVale = zeros(points^2);
q5 = 0;
i = 1;
for q1 = -90 :stepq1:90	% for q1
    for q2 = 0:step:135   % for q2
        for q3 = -145:step:0 % for q3
            for q4 = -90:step:90 % for q4
                    xwork(i) =  eval(subs(xt));
                    ywork(i) =  eval(subs(yt));
                    zwork(i) =  eval(subs(zt));
                    QVale(i) =  q2 + q3 + q4;
%                     if(zwork(i) < 1.74 & zwork(i) > 1.73)
%                           xwork(i)
%                           ywork(i)
%                           zwork(i)
%                           QVale(i)
%                           q1
%                           q2
%                           q3
%                           q4
%                     end
                    i = i+1;
            end
        end
   end
end
xwork
ywork
zwork
QVale
plot3(xwork, ywork, zwork, 'rx')
hold on
title('Workspace') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
axis equal
  
%% Plot the robotic arm, in at least different positions
 %%SOLVING FOR JOINT COORDINATES
figure (3) 
Task_old = [1.04168 -1.80423  0.090961  5;
        2.01691 -1.16446  1.48369   65;
        1.82893  0        2.34971   65;
        2.00515  1.15767  2.52676   5;
        0.989942 1.171463 0.624255 -55;
        0        2.08335  0.090961  5;]

Task = [0         -1.97988  0.624255  -25;
        0         -1.99239  2.17431   5;
        2.5        0        2.17431   25;
        0          1.99239  2.17431   5;
        0          1.97988  0.624255 -25;]
angles = zeros(5, 4);
for i = 1:5
    J_Cor = Joint_Coordinates(Task(i, 1), Task(i, 2), Task(i, 3), Task(i, 4), l1, l2, l3, l5);
    xx = [ J_Cor(1, 1); J_Cor(1, 2); J_Cor(1, 3); J_Cor(1, 4); J_Cor(1, 5)]; 
    yy = [ J_Cor(2, 1); J_Cor(2, 2); J_Cor(2, 3); J_Cor(2, 4); J_Cor(2, 5)];
    zz = [ J_Cor(3, 1); J_Cor(3, 2); J_Cor(3, 3); J_Cor(3, 4); J_Cor(3, 5)];
    angles(i, 1) = J_Cor(4, 1); angles(i, 2) = J_Cor(4, 2); angles(i, 3) = J_Cor(4, 3); angles(i, 4) = J_Cor(4, 4);
    plot3(xx, yy, zz, 'ko-','Linewidth', 2)
    axis equal
    hold on
    xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    T = "pos" + num2str(i, 1);
    text(xx(5,1) + 0.02, yy(5,1) + 0.02, zz(5,1) + 0.02, T);
    axis([-2.5 2.5 -2.5 2.5]);
    pause(1.5)
end

%% DISPLAY IK PLOT
figure(4)
disp("Display Inverse Kinematics Plot")
J_Cor = Joint_Coordinates(Task(2, 1), Task(2, 2), Task(2, 3), Task(2, 4), l1, l2, l3, l5);
    xx = [ J_Cor(1, 1); J_Cor(1, 2); J_Cor(1, 3); J_Cor(1, 4); J_Cor(1, 5)];
    yy = [ J_Cor(2, 1); J_Cor(2, 2); J_Cor(2, 3); J_Cor(2, 4); J_Cor(2, 5)];
    zz = [ J_Cor(3, 1); J_Cor(3, 2); J_Cor(3, 3); J_Cor(3, 4); J_Cor(3, 5)];
    plot3(xx, yy, zz, 'ko-','Linewidth', 4)
    hold on
    xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    T = num2str(J_Cor(4, 1), 3)
    text(xx(1,1) + 0.1, yy(1,1) + 0.1, zz(1,1) + 0.1, T);
    
    T = num2str(J_Cor(4, 2), 3);
    text(xx(2,1) + 0.1, yy(2,1) + 0.02, zz(2,1) + 0.1, T);

    T = num2str(J_Cor(4, 3), 3);
    text(xx(3,1) + 0.1, yy(3,1) + 0.1, zz(3,1) + 0.1, T);

    T = num2str(J_Cor(4, 4), 3);
    text(xx(4,1) + 0.1, yy(4,1) + 0.1, zz(4,1) + 0.1, T);

    T = "0";
    text(xx(5,1) + 0.1, yy(5,1) + 0.1, zz(5,1) + 0.1, T);
    axis([-2.5 2.5 -2.5 2.5]);

    %% IMPLEMENT FREE MOTION TRAJECTORY
    figure(5)
    disp("Display Free Joint space Plot")
    steps_to_take = 10;
    angle_incQ1 = (angles(2, 1) - angles(1, 1))/steps_to_take;
    angle_incQ2 = (angles(2, 2) - angles(1, 2))/steps_to_take;
    angle_incQ3 = (angles(2, 3) - angles(1, 3))/steps_to_take;
    angle_incQ4 = (angles(2, 4) - angles(1, 4))/steps_to_take;
    q1 = angles(1, 1);
    q2 = angles(1, 2);
    q3 = angles(1, 3);
    q4 = angles(1, 4);
    for i = 1:(steps_to_take+1)
        J_CorFK = Joint_CoordinatesFK(q1, q2, q3, q4, l1, l2, l3, l4);
        xx = [ J_CorFK(1, 1); J_CorFK(1, 2); J_CorFK(1, 3); J_CorFK(1, 4); J_CorFK(1, 5)];
        yy = [ J_CorFK(2, 1); J_CorFK(2, 2); J_CorFK(2, 3); J_CorFK(2, 4); J_CorFK(2, 5)];
        zz = [ J_CorFK(3, 1); J_CorFK(3, 2); J_CorFK(3, 3); J_CorFK(3, 4); J_CorFK(3, 5)];
        q1 = q1 + angle_incQ1;
        q2 = q2 + angle_incQ2;
        q3 = q3 + angle_incQ3;
        q4 = q4 + angle_incQ4;
        plot3(xx, yy, zz, 'o-','Linewidth', 1)
        axis equal
        hold on
        pause(1.5)
        xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    end
    
    steps_to_take = 10;
    angle_incQ1 = (angles(3, 1) - angles(2, 1))/steps_to_take;
    angle_incQ2 = (angles(3, 2) - angles(2, 2))/steps_to_take;
    angle_incQ3 = (angles(3, 3) - angles(2, 3))/steps_to_take;
    angle_incQ4 = (angles(3, 4) - angles(2, 4))/steps_to_take;
    
    for i = 1:(steps_to_take + 1)
        J_CorFK = Joint_CoordinatesFK(q1, q2, q3, q4, l1, l2, l3, l4);
        xx = [ J_CorFK(1, 1); J_CorFK(1, 2); J_CorFK(1, 3); J_CorFK(1, 4); J_CorFK(1, 5)];
        yy = [ J_CorFK(2, 1); J_CorFK(2, 2); J_CorFK(2, 3); J_CorFK(2, 4); J_CorFK(2, 5)];
        zz = [ J_CorFK(3, 1); J_CorFK(3, 2); J_CorFK(3, 3); J_CorFK(3, 4); J_CorFK(3, 5)];
        q1 = q1 + angle_incQ1;
        q2 = q2 + angle_incQ2;
        q3 = q3 + angle_incQ3;
        q4 = q4 + angle_incQ4;
        plot3(xx, yy, zz, 'o-','Linewidth', 1)
        axis equal
        hold on
        pause(1.5)
        xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    end

     steps_to_take = 10;
    angle_incQ1 = (angles(4, 1) - angles(3, 1))/steps_to_take;
    angle_incQ2 = (angles(4, 2) - angles(3, 2))/steps_to_take;
    angle_incQ3 = (angles(4, 3) - angles(3, 3))/steps_to_take;
    angle_incQ4 = (angles(4, 4) - angles(3, 4))/steps_to_take;
    
    for i = 1:(steps_to_take+1)
        J_CorFK = Joint_CoordinatesFK(q1, q2, q3, q4, l1, l2, l3, l4);
        xx = [ J_CorFK(1, 1); J_CorFK(1, 2); J_CorFK(1, 3); J_CorFK(1, 4); J_CorFK(1, 5)];
        yy = [ J_CorFK(2, 1); J_CorFK(2, 2); J_CorFK(2, 3); J_CorFK(2, 4); J_CorFK(2, 5)];
        zz = [ J_CorFK(3, 1); J_CorFK(3, 2); J_CorFK(3, 3); J_CorFK(3, 4); J_CorFK(3, 5)];
        q1 = q1 + angle_incQ1;
        q2 = q2 + angle_incQ2;
        q3 = q3 + angle_incQ3;
        q4 = q4 + angle_incQ4;
        plot3(xx, yy, zz, 'o-','Linewidth', 1)
        axis equal
        hold on
        pause(1.5)
        xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    end

    steps_to_take = 10;
    angle_incQ1 = (angles(5, 1) - angles(4, 1))/steps_to_take;
    angle_incQ2 = (angles(5, 2) - angles(4, 2))/steps_to_take;
    angle_incQ3 = (angles(5, 3) - angles(4, 3))/steps_to_take;
    angle_incQ4 = (angles(5, 4) - angles(4, 4))/steps_to_take;
    
    for i = 1:(steps_to_take+1)
        J_CorFK = Joint_CoordinatesFK(q1, q2, q3, q4, l1, l2, l3, l4);
        xx = [ J_CorFK(1, 1); J_CorFK(1, 2); J_CorFK(1, 3); J_CorFK(1, 4); J_CorFK(1, 5)];
        yy = [ J_CorFK(2, 1); J_CorFK(2, 2); J_CorFK(2, 3); J_CorFK(2, 4); J_CorFK(2, 5)];
        zz = [ J_CorFK(3, 1); J_CorFK(3, 2); J_CorFK(3, 3); J_CorFK(3, 4); J_CorFK(3, 5)];
        q1 = q1 + angle_incQ1;
        q2 = q2 + angle_incQ2;
        q3 = q3 + angle_incQ3;
        q4 = q4 + angle_incQ4;
        plot3(xx, yy, zz, 'o-','Linewidth', 1)
        axis equal
        hold on
        pause(1.5)
        xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    end
