
% The following code simulates the forward kinematics of a 5DOF Lynx robot
% serial manipulator. Figure 2 shows its movement from start to end
% position, Figure 1 shows the location of its end effector at points of
% its trajectory and Figure 3 shows the maximum potential workspace of the
% arm's end effector.

clear all
close all
clc
disp('The following code simulates the forward kinematics of a simple 2DOF')
disp('serial manipulator. Figure 2 shows its movement from start to end position,')
disp('Figure 1 shows the location of its end effector at points of its trajectory')
disp('and Figure 3 shows the maximum potential workspace of its end effector')

%% A series of joint angles
% The following variables are defined in the form of column-vectors with
% 4 rows each. Each row represents a different position (angle) of the joint.
% e.g. inititally we hae 60 degrees for q1 and -30 for q2.


%% Links Lengths
syms l1 l2 l3 l4 l5;


%% Trigonometric angles
syms q1 q2 q3 q4 q5;


%% distal table
T_ij = eye(4);

%% Tip position
% These equations are derived from the Forward Kinematic model of the 2DOF
% robot

a = [0 l2 l3 0 0]';
alpha = [90 0 0 -90 0]';
d = [l1 0 0 0 l5]';
theta = [q1 q2 q3 q4 q5]';

for i = 1:5
    T_ij = T_ij * Transformation(a(i), alpha(i), d(i), theta(i));
end

T_ij

xt = T_ij(1, 4)

yt = T_ij(2, 4)

zt = T_ij(3, 4)
l1 = 1;
l2 = 1;
l3 = 1;
l4 = 0;
l5 = 1;
pt = [ xt yt zt ] ;

% xt =cos(q1)*cos(q2)*l2 - l5*(cos((180*conj(q4))/pi)*(cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*sin((180*conj(q3))/pi) + cos((180*conj(q1))/pi)*cos((180*conj(q3))/pi)*sin((180*conj(q2))/pi)) + sin((180*conj(q4))/pi)*(cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*cos((180*conj(q3))/pi) - cos((180*conj(q1))/pi)*sin((180*conj(q2))/pi)*sin((180*conj(q3))/pi))) + cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*cos((180*conj(q3))/pi)*conj(l3) - cos((180*conj(q1))/pi)*sin((180*conj(q2))/pi)*sin((180*conj(q3))/pi)*conj(l3)
 

% subs(xt(1, 1))
% eval(subs(xt(1, 1)))



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the trajectory of the end-effector
%figure (1)
%set(1,'position',[680 558 560 420])

%plot3(pt(1,1),pt(1,2),0,'rx')       % plot the first position of the robot's end effector
%hold on
%plot(pt(2:4,1),pt(2:4,2),'x')       % plot the 3 following positions of the robot's end effector
%title('Tip Trajectory') ; xlabel('x (m)') ; ylabel('y (m)') ;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Workspace
step = 30; 
points = 360/step;

%% Plot the workspace of the robot
figure (3)
set(3,'position',[1243 190 560 420])

% xwork = zeros(1024, 1024) ; % reserving space for the variables, because
% ywork = zeros(1024, 1024) ; % otherwise they would be created later within a loop.
% zwork = zeros(1024, 1024) ; 
i = 1;
xwork = zeros(points^2) ; % reserving space for the variables, because
ywork = zeros(points^2) ; % otherwise they would be created later within a loop.
zwork = zeros(points^2) ; 
q5 = 0;
for q1 = -90:180:90	% for q1
    for q2 = 0:step:180   % for q2
        for q3 = -170:step:0  % for q3
            for q4 = 0:step:180 % for q4
                    xwork(i) =  eval(subs(xt));
                    ywork(i) =  eval(subs(yt));
                    zwork(i) =  eval(subs(zt));
                    i = i+1;
%                     plot(xwork,ywork, '.')
%                     title('X-Y') ; xlabel('x (m)') ; ylabel('y (m)')
%                     plot(ywork,zwork, 'rx')
%                     title('Y-Z') ; ylabel('y (m)') ; zlabel('z (m)')
%                     plot(xwork,zwork, '.')
%                     title('X-Z') ; xlabel('x (m)') ; zlabel('z (m)')
                    
%                     i = i+1;
            end
        end
   end
end

plot3(xwork,ywork,zwork, 'rx')
hold on
title('Workspace') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
axis equal
