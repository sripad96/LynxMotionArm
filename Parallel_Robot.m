%%
%%Assuming equal sides of base and platform

Base = 500;
Platform = 250;
Sa = 125;
L = 125;
syms xPos yPos a Theta1;

R =  Base/(2*cos(pi/6)); %Base_centre_to_vertex
r =  Platform/(2*cos(pi/6));%latform_centre_to_vertex

%%Link 1 Inverse Kinematics
phiDeg = a + 30;
PPy1 = yPos - (r*sin(phiDeg));
PPx1 = xPos - (r*cos(phiDeg));
PBy1 = 0;
PBx1 = 0;
c1 = atan2d(PPy1 - PBy1, PPx1 - PBx1);

Vertex_to_vertex1 = realsqrt((PPy1 - PBy1)^2 + (PPx1 - PBx1)^2);
dTop1 = Sa^2 - L^2 + Vertex_to_vertex1^2;
dBottom1 = 2*Sa*Vertex_to_vertex1;
d1 = acosd(dTop1 / dBottom1);
Theta1 = c1+d1;

%%Link 2 Inverse Kinematics
phiDeg2 = phiDeg + 120;
PPy2 = yPos - (r*sin(phiDeg2));
PPx2 = xPos - (r*cos(phiDeg2));
PBx2 = realsqrt(3)*R;
PBy2 = 0;
c2 = atan2d( (PPy2 - 0), (PPx2 - PBx2) );

Vertex_to_vertex2 = realsqrt((PPx2 - PBx2)^2 + (PPy2 - PBy2)^2);
dTop2 = Sa^2 - L^2 + Vertex_to_vertex2^2;
dBottom2 = 2*Sa * Vertex_to_vertex2;
d2 = acosd(dTop2 / dBottom2);
Theta2 = c2 - d2;


%%Link 3 Inverse Kinematics
phiDeg3 = phiDeg2 + 120;
PPy3 = yPos - (r*sin(phiDeg3));
PPx3 = xPos - (r*cos(phiDeg3));
PBy3 = realsqrt(3)*R/2;
PBx3 = 1.5*R;
c3 = atan2d((PPy3 - PBy3), (PPx3 - PBx3));
Vertex_to_vertex3 = realsqrt((PPx3 - PBx3)^2 + (PPy3 - PBy3)^2);
dTop3 = Sa^2 - L^2 + Vertex_to_vertex3^2;
dBottom3 = 2*Sa * Vertex_to_vertex3;
d3 = acosd(dTop3 / dBottom3);
    Theta3 = 360 + c3; %check equation