function [derivEulerAngles] = dEulerAngles(phi, theta, psi, p, q, r)

temp =   [1, sin(phi)*tan(theta), cos(phi)*tan(theta) ;
          0, cos(phi)           , -1*sin(phi)           ;
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

derivEulerAngles = temp*[p;q;r] ;

end