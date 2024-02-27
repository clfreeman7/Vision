clear all;
clc;
% Camera for tracking
cam = webcam(1);  %Initializing object for camera
cam.Resolution = cam.AvailableResolutions{6};   %Resolution 640x480

%Camera for good video
cam2nd = webcam(2);
cam2nd.Resolution = cam2nd.AvailableResolutions{6}; %Resolution 1920x1080

% Object for writing video
vwrite = VideoWriter('Lauren_test_3','MPEG-4');
vwrite_2ndcam = VideoWriter('Lauren_test_Real_3','MPEG-4');

open(vwrite);
open(vwrite_2ndcam);

% Preview cameras
preview(cam);
preview(cam2nd);

% Menu
choose = menu("Choose the resolution of the video",'640x480','1920x1080');

if choose == 1
%% Properties
cam = camera_properties_marker(cam);
cam.Resolution = cam.AvailableResolutions{1};   %Resolution 640x480
distortion = load('D:\Arun Niddish\Vision\Visual Tracking\Callibration images(640x480)\cameraParams');   %Parameters for distortion correction for 640x480
% Rot_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\640x480\Rotation_matrix')
% T_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\640x480\Translation_matrix')
pause(1.5);
end

if choose == 2
%% Properties
cam = camera_properties_marker_1920_Blue(cam);
cam.Resolution = cam.AvailableResolutions{6};   %Resolution 1920x1080
distortion = load('D:\Arun Niddish\Vision\Visual Tracking\Callibration images(1920x1080)\cameraParams')   %Parameters for distortion correction for 1920x1080
% Rot_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\1920x1080\Rotation_matrix')
% T_projection = load('D:\Arun Niddish\Vision\Visual Tracking\SE2\1920x1080\Translation_matrix')
pause(1.5);
end

input('Press <Enter> to begin video capture.');

% newim = cam.snapshot;
% imwrite(newim,'calib_xaxis.png');

%% Pushbutton
p = uipanel('Title','Press to stop tracking','FontSize',12);
buttonHandle = uicontrol(p,'Style', 'toggleButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)',...
                         'FontWeight','bold','Position',[230 100 100 22]);

%% Loop to capture the video
i = 1;
while true
    
    if ~ishandle(buttonHandle)
        disp('Tracking stopped by user');
        break;
    end
    
%     newim = cam.snapshot;
    [newim,ts] = cam.snapshot;
    newim = undistortImage(newim,distortion.cameraParams); 
    time_stamp(i,:) = ts;
    writeVideo(vwrite,newim);
    
%     newim_2ndcam = cam2nd.snapshot;
%     writeVideo(vwrite_2ndcam,newim_2ndcam);
    
    i = i+1;
end
close(vwrite);
close(vwrite_2ndcam);
