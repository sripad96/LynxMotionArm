clear all
close all
clc
disp('The following code simulates the INVERSE kinematics of a Parallel 3RRR robot')
%%
%%Assuming equal sides of base and platform

Base = 500;
Platform = 200;
Sa = 150;
L = 125;
syms xPos yPos a;

a = -10;
R =  Base/(2*cosd(30)); %Base_centre_to_vertex
r =  Platform/(2*cosd(30));%platform_centre_to_vertex
xPos = Base/2;
yPos = (Base/2)*tand(30);
%%Link 1 Inverse Kinematics
phiDeg = a + 30;
PPy1 = yPos - (r*sind(phiDeg));
PPx1 = xPos - (r*cosd(phiDeg));
PBy1 = 0;
PBx1 = 0;
c1 = atan2d(PPy1 - PBy1, PPx1 - PBx1);

Vertex_to_vertex1 = sqrt((PPy1 - PBy1)^2 + (PPx1 - PBx1)^2)
dTop1 = Sa^2 - L^2 + Vertex_to_vertex1^2;
dBottom1 = 2*Sa*Vertex_to_vertex1;
d1 = acosd(dTop1 / dBottom1);
Theta1 = c1+d1;
Sa1_x = Sa*cosd(Theta1);
Sa1_y = Sa*sind(Theta1);
distance1 = distBtwPnts(Sa1_x, Sa1_y, 0, PBx1, PBy1, 0, Sa);
if distance1 == false
    disp("OUT OF WORKSPACE ERROR 1")
    return
end
distance1 = distBtwPnts(Sa1_x, Sa1_y, 0, PPx1, PPy1, 0, L);
if distance1 == false
    disp("OUT OF WORKSPACE ERROR 2")
    return
end

%%Link 2 Inverse Kinematics
phiDeg2 = phiDeg + 120;
PPy2 = yPos - (r*sind(phiDeg2));
PPx2 = xPos - (r*cosd(phiDeg2));
PBx2 = Base;
PBy2 = 0;
c2 = atan2d( (PPy2 - 0), (PPx2 - PBx2) );

Vertex_to_vertex2 = sqrt((PPx2 - PBx2)^2 + (PPy2 - PBy2)^2)
dTop2 = Sa^2 - L^2 + Vertex_to_vertex2^2;
dBottom2 = 2*Sa * Vertex_to_vertex2;
d2 = acosd(dTop2 / dBottom2);
Theta2 = c2 - d2;
Sa2_x =  PBx2 + Sa*cosd(Theta2);
Sa2_y = Sa*sind(Theta2);
distance1 = distBtwPnts(Sa2_x, Sa2_y, 0, PBx2, PBy2, 0, Sa);
if distance1 == false
    disp("OUT OF WORKSPACE ERROR 3")
    return
end
distance1 = distBtwPnts(Sa2_x, Sa2_y, 0, PPx2, PPy2, 0, L);
if distance1 == false
    disp("OUT OF WORKSPACE ERROR 4")
    return
end

%%Link 3 Inverse Kinematics
phiDeg3 = phiDeg2 + 120;
PPy3 = yPos - (r*sind(phiDeg3));
PPx3 = xPos - (r*cosd(phiDeg3));
PBy3 = sqrt(Base^2 - (Base/2)^2)
PBx3 = Base/2;
c3 = atan2d((PPy3 - PBy3), (PPx3 - PBx3))
Vertex_to_vertex3 = sqrt((PPx3 - PBx3)^2 + (PPy3 - PBy3)^2)
dTop3 = Sa^2 - L^2 + Vertex_to_vertex3^2;
dBottom3 = 2*Sa * Vertex_to_vertex3;
d3 = acosd(dTop3 / dBottom3)
Theta3 = - d3 + c3 %check equation
Sa3_x = PBx3 + (Sa*cosd(Theta3));
Sa3_y = PBy3 + (Sa*sind(Theta3));
distance1 = distBtwPnts(Sa3_x, Sa3_y, 0, PBx3, PBy3, 0, Sa);
if distance1 == false
    disp("OUT OF WORKSPACE ERROR 5")
    return
end
distance1 = distBtwPnts(Sa3_x, Sa3_y, 0, PPx3, PPy3, 0, L);
if distance1 == false
    disp("OUT OF WORKSPACE ERROR 6")
    return
end

% distance1 = distBtwPnts(PPx2, PPy2, 0, PPx3, PPy3, 0, Platform);
% if distance1 == false
%     disp("ERRATIC EE")
%     return
% end
%% Plotting the robot
figure (10)
xx = [PBx1, PBx2, PBx3, PBx1];
yy = [PBy1, PBy2, PBy3, PBy1];
plot(xx, yy, 'ko-','Linewidth',1)
hold on
xx_s = [PPx1, PPx2, PPx3, PPx1];
yy_s = [PPy1, PPy2, PPy3, PPy1];
plot(xx_s, yy_s, 'ko-','Linewidth',1)
hold on
xx_sa1 = [PBx1, Sa1_x, PPx1];
yy_sa1 = [PBy1, Sa1_y, PPy1];
plot(xx_sa1, yy_sa1, 'ko-','Linewidth',1)
hold on
xx_sa2 = [PBx2, Sa2_x, PPx2];
yy_sa2 = [PBy2, Sa2_y, PPy2];
plot(xx_sa2, yy_sa2, 'ko-','Linewidth',1)

hold on
xx_sa3 = [PBx3, Sa3_x, PPx3];
yy_sa3 = [PBy3, Sa3_y, PPy3];
plot(xx_sa3, yy_sa3, 'ko-','Linewidth',1)
