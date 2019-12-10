function [cSigma, n1, n2, n3] = mixerMatrix(cp1, cp2, cp3, cp4)

% hi, anthony was here

[Jxx, Jzz, m, l, k] = quadcopterParameters();

mixer =   [ 1,  1,  1,  1;
            l, -l, -l,  l;
           -l, -l,  l,  l;
            k, -k,  k, -k];

outputs = mixer * [cp1; cp2; cp3; cp4];

cSigma = outputs(1);
n1 = outputs(2);
n2 = outputs(3);
n3 = outputs(4);

end

