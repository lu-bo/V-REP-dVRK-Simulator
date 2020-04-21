% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    %if (clientID>-1)
    disp('Connected to remote API server');
    [~,PSM_JOINT_ID(1,1)] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);
    [~,PSM_JOINT_ID(1,2)] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
    %[~,PSM_JOINT_ID(1,3)] = vrep.simxGetObjectHandle(clientID, 'J3_sx_TOOL1', vrep.simx_opmode_blocking);
    
    [~, position] = vrep.simxGetJointPosition(clientID, PSM_JOINT_ID(1,2), vrep.simx_opmode_blocking);
    
    [~] = vrep.simxSetObjectPosition(clientID, PSM_JOINT_ID(1,1), PSM_JOINT_ID(1,2), [0.3, 0.3, 0.3],vrep.simx_opmode_blocking)
    
    [~] = vrep.simxSetJointPosition(clientID, PSM_JOINT_ID(1,1), 0, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, PSM_JOINT_ID(1,2), 0, vrep.simx_opmode_blocking);
    %[~, Matrix] = vrep.simxGetJointPosition(clientID, double(PSM_JOINT_ID(1,2)), vrep.simx_opmode_blocking);
    
    [~, EulerAngles_1] = vrep.simxGetObjectOrientation(clientID, double(PSM_JOINT_ID(1,1)), double(PSM_JOINT_ID(1,2)), vrep.simx_opmode_blocking);
    %[~, EulerAngles_2] = vrep.simxGetObjectOrientation(clientID, double(PSM_JOINT_ID(1,3)), double(PSM_JOINT_ID(1,1)), vrep.simx_opmode_blocking);
    
    [~, Position]    = vrep.simxGetObjectPosition(clientID, double(PSM_JOINT_ID(1,1)), double(PSM_JOINT_ID(1,2)), vrep.simx_opmode_blocking);
   
    %target_Position = Position*1.0001;
    division_Position = (target_Position-Position)/100;
    
    target_Orientation_1 = EulerAngles_1*1.2;
    target_Orientation_2 = EulerAngles_2*1.2;
    division_Orientation_1 = (target_Orientation_1-EulerAngles_1)/100;
    division_Orientation_2 = (target_Orientation_2-EulerAngles_2)/100;
    
%%
dVRK_grasper('open')

%%
function simpleTest()

    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    %objectHandle = 80;
    %relativeToObjectHandle = -1;
    %simx_opmode_streaming = -1;

    if (clientID>-1)
        disp('Connected to remote API server');
        
        [~,PSM_JOINT_ID(1,1)] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
        [~,PSM_JOINT_ID(1,2)] = vrep.simxGetObjectHandle(clientID, 'J1_PSM2', vrep.simx_opmode_blocking);

        [~, EulerAngles] = vrep.simxGetObjectOrientation(clientID, double(PSM_JOINT_ID(1,1)), double(PSM_JOINT_ID(1,2)), vrep.simx_opmode_blocking);
        [~, Position]    = vrep.simxGetObjectPosition(clientID, double(PSM_JOINT_ID(1,1)), double(PSM_JOINT_ID(1,2)), vrep.simx_opmode_blocking);
        
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
            
        pause(0.2);
    
        % Now retrieve streaming data (i.e. in a non-blocking fashion):
        t=clock;
        startTime=t(6);
        currentTime=t(6);
        vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming); % Initialize streaming
        while (currentTime-startTime < 3)   
            [returnCode,data]=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer); % Try to retrieve the streamed data
            if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                fprintf('Mouse position x: %d\n',data); % Mouse position x is actualized when the cursor is over V-REP's window
            end
            t=clock;
            currentTime=t(6);
        end
            
        % Now send some data to V-REP in a non-blocking fashion:
        vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);

        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end
