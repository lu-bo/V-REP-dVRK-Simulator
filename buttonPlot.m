function buttonPlot
    fig_UI = uifigure('Name','Navigator');
    %pnl = uipanel(fig);
    
    btn_show_images = uibutton(fig_UI,'push', 'Text', 'Show Stereo Images', 'Position',[190, 350, 140, 30],...
               'ButtonPushedFcn', @(btn_show_images, event) button_show_images(btn_show_images));
    btn_close_images = uibutton(fig_UI,'push', 'Text', 'Close Images', 'Position',[450, 20, 100, 30],...
               'ButtonPushedFcn', @(btn_close_images, event) button_close_images(btn_close_images));
	btn_up = uibutton(fig_UI,'push', 'Text', 'Up', 'Position',[210, 250, 100, 22],...
               'ButtonPushedFcn', @(btn_up, event) button_up(btn_up));
    btn_down = uibutton(fig_UI,'push', 'Text', 'Down', 'Position',[210, 150, 100, 22],...
               'ButtonPushedFcn', @(btn_down, event) button_down(btn_down));
    btn_left = uibutton(fig_UI,'push', 'Text', 'Left', 'Position',[40, 200, 80, 22],...
               'ButtonPushedFcn', @(btn_left, event) button_left(btn_left));
    btn_right = uibutton(fig_UI,'push', 'Text', 'Right', 'Position',[410, 200, 80, 22],...
               'ButtonPushedFcn', @(btn_right, event) button_right(btn_right));
    btn_clock_wise = uibutton(fig_UI,'push', 'Text', 'Rotate: Clockwise', 'Position',[120, 200, 130, 22],...
               'ButtonPushedFcn', @(btn_clock_wise, event) botton_clock_wise(btn_clock_wise));       
    btn_anti_clock_wise = uibutton(fig_UI,'push', 'Text', 'Rotate: Anti Clockwise', 'Position',[290, 200, 140, 22],...
               'ButtonPushedFcn', @(btn_anti_clock_wise, event) botton_anti_clock_wise(btn_anti_clock_wise)); 
    btn_zoom_in = uibutton(fig_UI,'push', 'Text', 'Zoom In', 'Position',[210, 275, 100, 22],...
               'ButtonPushedFcn', @(btn_zoom_in, event) botton_zoom_in(btn_zoom_in));  
    btn_zoom_out = uibutton(fig_UI,'push', 'Text', 'Zoom Out', 'Position',[210, 128, 100, 22],...
               'ButtonPushedFcn', @(btn_zoom_out, event) botton_zoom_out(btn_zoom_out));  
    
end

function [image_left, image_right] = dVRK_get_stereo_vision
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~, left_visionSensor]  = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_left', vrep.simx_opmode_blocking);
    [~, right_visionSensor] = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_right', vrep.simx_opmode_blocking);
    
    [~, ~,  image_left] = vrep.simxGetVisionSensorImage2(clientID, left_visionSensor,  0, vrep.simx_opmode_blocking); % '0' indicates -> show RGB image.
    [~, ~, image_right]= vrep.simxGetVisionSensorImage2(clientID, right_visionSensor, 0, vrep.simx_opmode_blocking);
    image_left = flip(image_left, 1); image_right = flip(image_right, 1);
    
    subplot(1, 2, 1); imshow(image_left); subplot(1, 2 ,2); imshow(image_right);
    vrep.delete(); % call the destructor!
end

% Create the function for the ButtonPushedFcn callback
% including pitch, Roll, and Yaw 
function button_show_images(btn_show_images)
    figure; title('Stereo Images from dVRK Vision Sensors')
    dVRK_get_stereo_vision;
end

function button_close_images(btn_close_images)
    close all;
    %delete(fig_UI);
end


function button_up(btn_up)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J2_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J2_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J2_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J2_TOOL1, jointPosition + 2*3.14/360, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function button_down(btn_down)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J2_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J2_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J2_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J2_TOOL1, jointPosition - 2*3.14/360, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function button_left(btn_left)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J1_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J1_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J1_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J1_TOOL1, jointPosition - 2*3.14/360, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function button_right(btn_right)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J1_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J1_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J1_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J1_TOOL1, jointPosition + 2*3.14/360, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function botton_clock_wise(btn_clock_wise)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J4_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J4_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J4_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J4_TOOL1, jointPosition - 2*3.14/360, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function botton_anti_clock_wise(btn_anti_clock_wise)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J4_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J4_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J4_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J4_TOOL1, jointPosition + 2*3.14/360, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function botton_zoom_in(btn_zoom_in)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J3_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J3_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J3_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J3_TOOL1, jointPosition + 0.005, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end

function botton_zoom_out(btn_zoom_out)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J3_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J3_ECM', vrep.simx_opmode_blocking);
    [~, jointPosition] = vrep.simxGetJointPosition(clientID, J3_TOOL1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J3_TOOL1, jointPosition - 0.005, vrep.simx_opmode_blocking);
    
    dVRK_get_stereo_vision; vrep.delete(); % call the destructor!
end
