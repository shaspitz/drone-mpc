function [TM] = eulerAngles2TM(phi, theta, psi)

[TM] = [cos(psi)*cos(theta)                             , sin(psi)*cos(theta)                             , -sin(theta)        ;
        cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), cos(theta)*sin(phi);
        cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi), cos(theta)*cos(phi)];

end