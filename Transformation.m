function T_ij = Transformation(a, alpha, d, theta)
     calpha = cosd(alpha);
     salpha = sind(alpha);
     ctheta = cosd(theta);
     stheta = sind(theta);
     
     T_ij = [ctheta -(calpha*stheta) (salpha*stheta) (a*ctheta); 
             stheta  (calpha*ctheta) -(salpha*ctheta) (a*stheta);
             0       (salpha)          calpha          d;
             0       0                 0               1;]

end