clear all;
clc;

% Adding dependencies
% addpath('../');

% Video file information
input_vid_dir = 'C:\Users\anmahendran\Documents\Micro Spines';
vid_name = 'Lauren_test_3.mp4';
input_vid_filename = fullfile(input_vid_dir,vid_name);
output_vid_filename = 'Lauren_test_3_gcf.mp4';

% Instantiate a video objects for this video.
params.vread = VideoReader(input_vid_filename);
params.vwrite = VideoWriter(output_vid_filename,'MPEG-4');
open(params.vwrite);

%Starting Frame
% start_frame = 1;

% Tracking parameters
params.number_of_markers = 3;

% Overlay Image
% overlay_img = imread('World Scenario/World scenario 1 undistorted.png');
% params.overlay_img_cut = imcrop(overlay_img,[110,40,1920,1080]);

% Choose overlay image (Yes/No - 1/0)
params.overlay = 0;

% Initilaize
tracker_obj = OfflineTracking(params);

% Call tracking function 
% [output_data,tracking_data_centroid] =  tracker_obj.tracking();
[output_data] =  tracker_obj.tracking();
ts = load(fullfile(input_vid_dir,'Lauren_test_3_time_stamp.mat'));
tracking_data = [output_data ts.time_stamp];

% Finding the start frame
start_frame_actual = tracker_obj.find_start_frame() - 1;

% Adjusting time stamp with starting frame as t = 0
tracking_data(:,34) = tracking_data(:,34) - tracking_data(start_frame_actual,34);

% Gait seperation:

gait_cycle_duration = 1.1;

ii = 1;

for jj = start_frame_actual:size(tracking_data,1)

    if tracking_data(jj,34) < gait_cycle_duration*ii
       tracking_data(jj,35) = ii;
    end
    
    if tracking_data(jj,34) > gait_cycle_duration*ii
       tracking_data(jj,35) = ii;
       ii = ii+1;
    end
end
