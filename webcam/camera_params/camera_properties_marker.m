% This is a function to setup the camera properties to capture markers.

function cam = camera_properties_marker(cam)

%% Properties
cam.Resolution = cam.AvailableResolutions{1}; %Resolution - {1}-640 x 480 {6}-1920x1080
cam.BacklightCompensation = 98;
cam.Brightness = 50;
cam.Contrast = 35;
cam.ExposureMode = 'manual';
cam.Exposure = -7;
cam.Gain = 36;
cam.Gamma = 100;
cam.Hue = 0;
cam.Iris = 0;
cam.Saturation = 88;
cam.Sharpness = 5;
cam.WhiteBalanceMode = 'auto';
cam.WhiteBalance = 4000;

end