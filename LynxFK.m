
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
%% Trigonometric angles
syms q1 q2 q3 q4 q5;


%% distal table
% T_ij = eye(4);

%% Tip position
% These equations are derived from the Forward Kinematic model of the 2DOF
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

xt = T_ij(1, 4)

yt = T_ij(2, 4)

zt = T_ij(3, 4)

% xt =cos(q1)*cos(q2)*l2 - l5*(cos((180*conj(q4))/pi)*(cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*sin((180*conj(q3))/pi) + cos((180*conj(q1))/pi)*cos((180*conj(q3))/pi)*sin((180*conj(q2))/pi)) + sin((180*conj(q4))/pi)*(cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*cos((180*conj(q3))/pi) - cos((180*conj(q1))/pi)*sin((180*conj(q2))/pi)*sin((180*conj(q3))/pi))) + cos((180*conj(q1))/pi)*cos((180*conj(q2))/pi)*cos((180*conj(q3))/pi)*conj(l3) - cos((180*conj(q1))/pi)*sin((180*conj(q2))/pi)*sin((180*conj(q3))/pi)*conj(l3)

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
stepq1 = 30; 
step = 60;
points = 360/step;

%% Plot the workspace of the robot
figure (3)
set(3,'position',[1243 190 560 420])
i = 1;
xwork = zeros(points^2) ; % reserving space for the variables, because
ywork = zeros(points^2) ; % otherwise they would be created later within a loop.
zwork = zeros(points^2) ;
QVale = zeros(points^2);
q5 = 0;

for q1 = -90 :stepq1:90	% for q1
    for q2 = 0:step:180   % for q2
        for q3 = 0:step:180 % for q3
            for q4 = 0:step:180 % for q4
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

% xwork
% ywork
% zwork
% QVale

plot3(xwork,ywork,zwork, 'rx')
hold on
title('Workspace') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
axis equal


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Inverse Kinematics Section

xinvMat = [-0.781237  -0.826955  -0.477442   0          0.462012   0.950612]';
yinvMat = [ 0         -0.477442  -0.826955  -0.954885  -0.800229  -0.548836]';
zinvMat = [0.530861    1.51567    1.51567    1.51567    1.56912    0.584313]';
QinvMat = [130         70         70         70        -50        -110]';

figure (4)
set(4,'position',[1200 190 560 420])

plot3(xinvMat, yinvMat, zinvMat, 'rx')
hold on
title('Workspace') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;

%%
%%Solve the Inverse Kinematics Model
syms xinv yinv zinv Qinv;
Theta1 = atan2d(yinv, xinv)

R = l5*cosd(Qinv);
xPrime = xinv - (R*cosd(Theta1));
yPrime = yinv - (R*sind(Theta1));
zPrime = zinv - (l5 * sind(Qinv));

rPrime = sqrt(xinv^2 + yinv^2) - R
N = sqrt((zPrime - l1)^2 + rPrime^2);
gamma = atan2d((zPrime - l1), rPrime);
Theta2_a = gamma - acosd((N^2 + l2^2 - l3^2) / (2*l2*N))
Theta2_b = gamma + acosd((N^2 + l2^2 - l3^2) / (2*l2*N))

Theta3_a = acosd((N^2 - l2^2 - l3^2)/(2*l2*l3))
Theta3_b = -1*acosd((N^2 - l2^2 - l3^2)/(2*l2*l3))
Theta4_a = Qinv - Theta2_a - Theta3_a;
Theta4_b = Qinv - Theta2_b - Theta3_b;



    i = 2;
%     zinv = zinvMat(i);
    zinv = 4;
%     yinv = yinvMat(i);
    yinv = 0;
%     xinv = xinvMat(i)
    xinv = 0;
%     Qinv = QinvMat(i);
    Qinv = 90;
    disp("INVERSE KINEMATICS")
    q1 = eval(subs(Theta1))
    q2 = eval(subs(Theta2_a))
    q3 = eval(subs(Theta3_a))
    q4 = eval(subs(Theta4_a))
    disp("Solution B")
    q1 = eval(subs(Theta1))
    q2 = eval(subs(Theta2_b))
    q3 = eval(subs(Theta3_b))
    q4 = eval(subs(Theta4_b))
    


q1 = 0;    
q2 = 0;
% q2 = 180;
q3 = 0;
% q3 = -170;
q4 = 0;
% q4 = 120;
q5 = 0;

xsolution = eval(subs(xt))
ysolution = eval(subs(yt))
zsolution = eval(subs(zt))


   
%% Plot the robotic arm, in 6 different positions
 %%SOLVING FOR JOINT COORDINATES
figure (2) 



J0 = zeros(6,3);
J1 = zeros(6,3);
J2 = zeros(6,3);
J3 = zeros(6,3);
J4 = zeros(6,3);
J5 = zeros(6,3);
