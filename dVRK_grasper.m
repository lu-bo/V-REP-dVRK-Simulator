function dVRK_grasper(com)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    [~,J2_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);
    [~,J3_dx_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J3_dx_TOOL1', vrep.simx_opmode_blocking);
    [~,J3_sx_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J3_sx_TOOL1', vrep.simx_opmode_blocking);
    
    [~, J3_dx_TOOL1_EulerAngles] = vrep.simxGetObjectOrientation(clientID, J3_dx_TOOL1, J2_TOOL1, vrep.simx_opmode_blocking);
    [~, J3_sx_TOOL1_EulerAngles] = vrep.simxGetObjectOrientation(clientID, J3_sx_TOOL1, J2_TOOL1, vrep.simx_opmode_blocking);
    
    step_num = 50;
    % ---------------------------------------------------------------------
    % ---------------------------------------------------------------------
    % -----------------depend on initial positions
    J3_dx_TOOL1_EulerAngles_target = J3_dx_TOOL1_EulerAngles * 1.2;
    J3_sx_TOOL1_EulerAngles_target = J3_sx_TOOL1_EulerAngles * 1.2;
    % ---------------------------------------------------------------------
    % ---------------------------------------------------------------------
    
    J3_dx_TOOL1_EulerAngles_division = (J3_dx_TOOL1_EulerAngles_target - J3_dx_TOOL1_EulerAngles) / step_num;
    J3_sx_TOOL1_EulerAngles_division = (J3_sx_TOOL1_EulerAngles_target - J3_sx_TOOL1_EulerAngles) / step_num;
    
    if strcmp(com,'open')
        for i = 1 : step_num
        [~] = vrep.simxSetObjectOrientation(clientID, J3_dx_TOOL1, J2_TOOL1, J3_dx_TOOL1_EulerAngles+J3_dx_TOOL1_EulerAngles_division*i, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetObjectOrientation(clientID, J3_sx_TOOL1, J2_TOOL1, J3_sx_TOOL1_EulerAngles+J3_sx_TOOL1_EulerAngles_division*i, vrep.simx_opmode_blocking);
        end  
    end 
    if strcmp(com,'close')
        for i = 1:50
        [~] = vrep.simxSetObjectOrientation(clientID, J3_dx_TOOL1, J2_TOOL1, J3_dx_TOOL1_EulerAngles-J3_dx_TOOL1_EulerAngles_division*i, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetObjectOrientation(clientID, J3_sx_TOOL1, J2_TOOL1, J3_sx_TOOL1_EulerAngles-J3_sx_TOOL1_EulerAngles_division*i, vrep.simx_opmode_blocking);
        end  
    end 
    
    vrep.delete(); % call the destructor!
end
%%

