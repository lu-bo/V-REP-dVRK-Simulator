n=20;% image total number
Hmarker2world = cell(1,n);
Hgrid2cam = cell(1,n);
for ii=1:n
    Data1 = textscan('Hmarker2world', ii, 'txt', '%f');
    Hmarker2world{ii}= Data1;
    Data2 = textscan('Hgrid2cam', ii, 'txt', '%f');
    Hgrid2cam{ii}= Data2;
end
[Hcam2marker_, err] = TSAIleastSquareCalibration(Hmarker2world, Hgrid2cam);