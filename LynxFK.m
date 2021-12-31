
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
T_ij1 = Transformation(a(1), alpha(1), d(1), theta(1));
T_ij2 = Transformation(a(2), alpha(2), d(2), theta(2));
T_ij3 = Transformation(a(3), alpha(3), d(3), theta(3));
T_ij4 = Transformation(a(4), alpha(4), d(4), theta(4));
T_ij5 = Transformation(a(5), alpha(5), d(5), theta(5));

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

% xt =cos(q1)*cos(q2)*l2 - l5*(cos((180*conj(q4))/pi)*(cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*sin((180*conj(q3))/pi) + cos((180*conj(q1))/pi)*cos((180*conj(q3))/pi)*sin((180*conj(q2))/pi)) + sin((180*conj(q4))/pi)*(cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*cos((180*conj(q3))/pi) - cos((180*conj(q1))/pi)*sin((180*conj(q2))/pi)*sin((180*conj(q3))/pi))) + cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*cos((180*conj(q3))/pi)*conj(l3) - cos((180*conj(q1))/pi)*sin((180*conj(q2))/pi)*sin((180*conj(q3))/pi)*conj(l3)

%% Workspace
stepq1 = 10; 
step = 10;
points = 180/step;

%% Plot the workspace of the robot
figure (3)
set(3,'position',[1243 190 560 420])
i = 1;
xwork = zeros(points^2) ; % reserving space for the variables, because
ywork = zeros(points^2) ; % otherwise they would be created later within a loop.
zwork = zeros(points^2) ;
QVale = zeros(points^2);
q5 = 0;

% for q1 = -90 :stepq1:90	% for q1
%     for q2 = 0:step:135   % for q2
%         for q3 = -145:step:0 % for q3
%             for q4 = -90:step:90 % for q4
%                     xwork(i) =  eval(subs(xt));
%                     ywork(i) =  eval(subs(yt));
%                     zwork(i) =  eval(subs(zt));
%                     QVale(i) =  q2 + q3 + q4;
% %                     if(zwork(i) < 1.74 & zwork(i) > 1.73)
% %                           xwork(i)
% %                           ywork(i)
% %                           zwork(i)
% %                           QVale(i)
% %                           q1
% %                           q2
% %                           q3
% %                           q4
% %                     end
%                     i = i+1;
%             end
%         end
%    end
% end
                          xwork
                          ywork
                          zwork
                          QVale
plot3(xwork,ywork,zwork, 'rx')
hold on
title('Workspace') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
axis equal


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inverse Kinematics Section

q1 = 0;    
q2 = 0;
% q2 = 180;
q3 = 90;
% q3 = -170;
q4 = -150;
% q4 = 120;
q5 = 0;

xsolution = eval(subs(xt))
ysolution = eval(subs(yt))
zsolution = eval(subs(zt))


   
%%Plot the robotic arm, in at least different positions
 %%SOLVING FOR JOINT COORDINATES
figure (2) 


Task = [1.04168 -1.80423  0.090961  5;
        2.01691 -1.16446  1.48369   65;
        1.82893  0        2.34971   65;
        2.00515  1.15767  2.52676   5;
        0.989942 1.171463 0.624255 -55;
        0        2.08335  0.090961  5;]

for i = 1:6
    J_Cor = Joint_Coordinates(Task(i, 1), Task(i, 2), Task(i, 3), Task(i, 4), l1, l2, l3, l5);
    xx = [ J_Cor(1, 1); J_Cor(1, 2); J_Cor(1, 3); J_Cor(1, 4); J_Cor(1, 5)]; 
    yy = [ J_Cor(2, 1); J_Cor(2, 2); J_Cor(2, 3); J_Cor(2, 4); J_Cor(2, 5)];
    zz = [ J_Cor(3, 1); J_Cor(3, 2); J_Cor(3, 3); J_Cor(3, 4); J_Cor(3, 5)]; 
    plot3(xx, yy, zz, 'ko-','Linewidth', 2)
    axis equal
    hold on
    xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    T = num2str(xx(1,1), 5) + ", " + num2str(yy(1,1), 5) + ", " + num2str(zz(1,1), 5);
    text(xx(1,1) + 0.02, yy(1,1) + 0.02, zz(1,1) + 0.02, T);
    axis([-4 4 -4 4]);
    pause(1.5)
    hold off
    pause(0.1)
end


% text(pt(4,1),pt(4,2),'x'); 
% text(pt(4,1) + 0.002,pt(4,2) + 0.002,'ptEnd');

disp("Display Inverse Kinematics Plot")
J_Cor = Joint_Coordinates(Task(2, 1), Task(2, 2), Task(2, 3), Task(2, 4), l1, l2, l3, l5);
    xx = [ J_Cor(1, 1); J_Cor(1, 2); J_Cor(1, 3); J_Cor(1, 4); J_Cor(1, 5)] 
    yy = [ J_Cor(2, 1); J_Cor(2, 2); J_Cor(2, 3); J_Cor(2, 4); J_Cor(2, 5)] 
    zz = [ J_Cor(3, 1); J_Cor(3, 2); J_Cor(3, 3); J_Cor(3, 4); J_Cor(3, 5)] 
    plot3(xx, yy, zz, 'ko-','Linewidth', 2)
    hold on
    xlabel('x (units)') ; ylabel('y (units)'); zlabel('z (units)');
    T = num2str(xx(1,1), 5) + ", " + num2str(yy(1,1), 5) + ", " + num2str(zz(1,1), 5);
    text(xx(1,1) + 0.02, yy(1,1) + 0.02, zz(1,1) + 0.02, T);

    T = num2str(xx(2,1), 5) + ", " + num2str(yy(2,1), 5) + ", " + num2str(zz(2,1), 5);
    text(xx(2,1) + 0.02, yy(2,1) + 0.02, zz(2,1) + 0.02, T);

    T = num2str(xx(3,1), 5) + ", " + num2str(yy(3,1), 5) + ", " + num2str(zz(3,1), 5);
    text(xx(3,1) + 0.02, yy(3,1) + 0.02, zz(3,1) + 0.02, T);

    T = num2str(xx(4,1), 5) + ", " + num2str(yy(4,1), 5) + ", " + num2str(zz(4,1), 5);
    text(xx(4,1) + 0.02, yy(4,1) + 0.02, zz(4,1) + 0.02, T);

    T = num2str(xx(5,1), 5) + ", " + num2str(yy(5,1), 5) + ", " + num2str(zz(5,1), 5);
    text(xx(5,1) + 0.02, yy(5,1) + 0.02, zz(5,1) + 0.02, T);
%     text(xx(1) + 0.02, yy(1,1) + 0.02, zz(1,1) + 0.02, '');
