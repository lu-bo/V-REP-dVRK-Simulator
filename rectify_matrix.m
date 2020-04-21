%%
T = stereoParams.TranslationOfCamera2';
R = stereoParams.RotationOfCamera2;
e_1 = T / norm(T);
e_2 = [-T(2) T(1) 0]'/(T(1)^2 + T(2)^2)^0.5;
e_3 = cross(e_1, e_2);

R_rect = [e_1' ; e_2' ; e_3'];

R_L = sqrtm(R);
R_R = inv(R_L);

R_rect*R_L;
R_rect*R_R