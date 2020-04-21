function [RotationMatrix_leftVision_RCM, ...
          Position__leftVision_RCM, ...
          Transformed_coordinates] = ...
          dVRK_get_transformation_relationship_leftVision_to_visionRCM(Coordinates_in_leftVision)
    % the returned rotation and translation matrix should be applied to the
    % data with unit of 'meter'!!! 

    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    %if (clientID>-1)
    
    [~, left_vision_sensor] = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_left', vrep.simx_opmode_blocking);
    [~, RCM_vision_system]  = vrep.simxGetObjectHandle(clientID, 'J1_ECM', vrep.simx_opmode_blocking);

    
    %[~, matrix]=vrep.simxGetJointMatrix(clientID, RCM_vision_system, vrep.simx_opmode_blocking)
    
    [~, EulerAngles_leftVision_RCM] = vrep.simxGetObjectOrientation(clientID, left_vision_sensor, RCM_vision_system, vrep.simx_opmode_blocking);
    [~, Position__leftVision_RCM]    = vrep.simxGetObjectPosition(clientID, left_vision_sensor, RCM_vision_system, vrep.simx_opmode_blocking);
    
    R_X = [1 0 0; 0 cos(EulerAngles_leftVision_RCM(1)) -sin(EulerAngles_leftVision_RCM(1)); 0 sin(EulerAngles_leftVision_RCM(1)) cos(EulerAngles_leftVision_RCM(1))];
    R_Y = [cos(EulerAngles_leftVision_RCM(2)) 0 sin(EulerAngles_leftVision_RCM(2)); 0 1 0; -sin(EulerAngles_leftVision_RCM(2)) 0 cos(EulerAngles_leftVision_RCM(2))];
    R_Z = [cos(EulerAngles_leftVision_RCM(3)) -sin(EulerAngles_leftVision_RCM(3)) 0; sin(EulerAngles_leftVision_RCM(3)) cos(EulerAngles_leftVision_RCM(3)) 0; 0 0 1];
    
    RotationMatrix_leftVision_RCM = R_X*R_Y*R_Z;
    
    % --------------- 3D coordinates transformation -----------------------
    % The unit of the coordinates should be 'm', be cautious!
    Regularized_Coordinates = Coordinates_in_leftVision * 0.001;
        
    Transformation_Matrix = [RotationMatrix_leftVision_leftArmRCM ...
                             Position__leftVision_leftArmRCM'; ...
                             0 0 0 1];
      
    Transformed_coordinates = Transformation_Matrix * ...
                              [Regularized_Coordinates'; ...
                              repmat(1, 1, size(Regularized_Coordinates, 1))];
                          
    Transformed_coordinates = Transformed_coordinates(1 : 3, :)';

    vrep.delete(); % call the destructor!
end