addpath(genpath ('./'));
nn = 15;% image total number
Hmarker2world = zeros(4,4,nn);
Hgrid2cam = zeros(4,4,nn);
count = 1;
% PoseName = ["1", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "2", "20", "3", "4", "5", "6", "7", "8", "9"];
PoseName = ["1", "10", "11", "12", "13", "14", "15", "2", "3", "4", "5", "6", "7", "8", "9"];
% PoseName = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"];

path_preNmae = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\Postdoc_works\2019-second_half\Research_works\Suturing\Paper_preparation\Experimental figures\suture_grasping\Exp_6_surgical_phantom\cal\';
for ii = 1:nn
    filename = path_preNmae + "RobotPose" + PoseName(ii) + ".txt";
    fid = fopen(filename);
    Data1 = textscan(fid, '%f%f%f%f');
    Mat1 = cell2mat(Data1);
    Hmarker2world(:,:,count)= inv(Mat1);
    R = stereoParams.CameraParameters1.RotationMatrices(:,:,count)';
    P = stereoParams.CameraParameters1.TranslationVectors(count,:)' / 1000;
    Mat2 = [R P; 0 0 0 1];
    Hgrid2cam(:,:,count)= Mat2;
    count = count + 1;
    fclose(fid);
end
[Hcam2marker_, err] = TSAIleastSquareCalibration(Hmarker2world, Hgrid2cam);
Hcam2marker_