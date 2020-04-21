
%%
video = VideoWriter('suture_video_labeling_1.avi'); %create the video object
video.FrameRate = 5; 
open(video); %open the file for writing

for ii=1:500
  [left_origin, right_origin] = dVRK_virtual_sensor.get_stereo_vision;
  %I = imread('the ith image.jpg'); %read the next image
  writeVideo(video, left_origin); %write the image to file
end
close(video); %close the file

display("Finish");

