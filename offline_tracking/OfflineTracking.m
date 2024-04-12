classdef OfflineTracking < handle
    properties
        % Robot and tracking information that can be defined
        n_markers;      % number of flourescent markers on robot

        marker_size;    % minimum size of marker in pixels (integer)

        n_frames;       % number of frames in video 

        max_dist;       % maximum distance a marker can travel in a single frame

        start_frame;

        % Stored tracking data
        centroids;
        prev_pos;
        P0;
        this_pos;
        cent;
        tracking_data_centroid; 

        % Video variables
        vread;
        vwrite;

    end
    methods
        function obj = OfflineTracking(params)
            % Set values.
            obj.n_markers = params.number_of_markers;
            obj.vread = params.vread;
            obj.n_frames = obj.vread.NumberOfFrame;
            obj.vwrite = params.vwrite;

            obj.set_property(params, 'start_frame', 1.0);
            obj.set_property(params, 'marker_size', 10);
            obj.set_property(params, 'max_dist',60);
            obj.set_property(params, 'P0', []);

            % Initialize stored tracking data.
            obj.prev_pos = [];
            obj.this_pos = [];
            obj.cent = [];
            obj.tracking_data_centroid = [];  
            obj.centroids = zeros(obj.n_frames,3*obj.n_markers);

        end

        % Make function to set properties if they are passed as an
        % input. Otherwise, use default values.
        function set_property(obj, source_struct, param_name, def_val)
            if ( isfield(source_struct, param_name) )
                obj.(param_name) = source_struct.(param_name);
            else
                obj.(param_name) = def_val;
            end
        end

        % Tracking function.
        function [tracking_data] = tracking(obj)
            for i_frame = 1:obj.n_frames
                this_frame = read(obj.vread,i_frame);
                % Input mask (binarize) function based on lighting/ environment.
                frame_mask =  createMaskhdblue(this_frame);

                % Isolate the markers based on connected components of at
                % least this.marker_size number of pixels.
                frame_mask = bwareaopen(frame_mask,obj.marker_size);

                % Flood fill markers.
                frame_mask = imfill(frame_mask, 'holes');

                % Label the markers and extract marker properties.
                [labeledImage, n_markers_detected] = bwlabel(frame_mask);
                stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area','EquivDiameter');

                % Initialize marker centroids (2-D). 
                obj.cent = zeros(n_markers_detected,2);
 
                for i_marker = 1:n_markers_detected
                    % Store marker centroids.
                    obj.cent(i_marker,:) = stats(i_marker).Centroid;

                    % Correct y-axis values (flipped).
                    obj.cent(i_marker,2) = 1080 - obj.cent(i_marker,2) ;  
                end

                % Initialize data as 3-D points.
                obj.cent = [obj.cent,zeros(n_markers_detected,1);];
                if i_frame == 1
                    if isempty(obj.P0)
                        obj.P0 = obj.cent;
                    else
                        obj.cent = obj.P0;
                    end
                    obj.prev_pos = obj.cent;
                    obj.centroids(i_frame,:) = reshape(obj.prev_pos',1,[]);
                    figure
                    plot(obj,this_frame,n_markers_detected,i_frame);

             
                else
                    % Find markers' nearest neighbor for data alignment.
                    obj.this_pos = nearest_neighbor(obj,n_markers_detected);

                    % Find local rigid-body transformation of robot from 
                    % previous frame.
                    [Rot,T] = pose_estimation(obj,obj.this_pos,obj.prev_pos);
                    theta(i_frame,:) = reshape(Rot,[1,9]);
                    trans(i_frame,:) = T';

                    % Find global rigid-body transformation of robot from 
                    % previous frame.
                    [Rot,T] = pose_estimation(obj,obj.P0,obj.this_pos);
                    theta_G(i_frame,:) = reshape(Rot,[1,9]);
                    trans_G(i_frame,:) = T';

                    obj.prev_pos = obj.this_pos;
                    obj.centroids(i_frame,:) = reshape(obj.prev_pos',1,[]);

                    plot(obj,this_frame,n_markers_detected,i_frame);

                end

            end

            tracking_data = cat(2,obj.centroids,theta,trans,theta_G,trans_G);
            close(obj.vwrite);
        end


        % Find markers' nearest neighbor in next frame to ensure marker
        % data points are matched to correct markers. 
     function centroids = nearest_neighbor(obj,n_markers_detected)      %Change the name of the centroid ->
            % Initialize current marker positions.
            obj.this_pos = zeros(obj.n_markers,3);

             % Match markers in this frame to markers in previous frame by
             % finding the minimum euclidean distance between frames
             % (bounded by a max allowable distance).
            if n_markers_detected > obj.n_markers
                for i = 1:obj.n_markers
                    for j = 1:n_markers_detected
                        X = [obj.prev_pos(i,:); obj.cent(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);  
                    if dmin < obj.max_dist
                        obj.this_pos(i,:) = obj.cent(ind,:);
                    end
                end
            else
                for i = 1:n_markers_detected
                    for j = 1:obj.n_markers
                        X = [obj.cent(i,:); obj.prev_pos(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);
                    if dmin < obj.max_dist
                        obj.this_pos(ind,:) = obj.cent(i,:);
                    end
                end
            end
            
            clear d;
            % Find which marker data are missing.
            missing_markers = find(obj.this_pos(:,1) == 0); 
            
            % Fill in missing marker data if needed.
            if isempty(missing_markers)
                centroids = obj.this_pos;
            else
                centroids = occlusion(obj,missing_markers);
            end

        end 

        % Reconstruct the missing marker data in case of tether occlusion. 
        function centroids = occlusion(obj,missing_markers)   
            newPrevPt = obj.prev_pos;
            newPrevPt(missing_markers,:) = 0;

        %   Find rigid transformation relative to previous frame based on
        %  limited marker data (by removing missing marker data).
            [Rot,T] = pose_estimation(obj,newPrevPt,obj.this_pos);

            % Reconstruct the missing data.
            for i = missing_markers
                obj.this_pos(missing_markers,:) = Rot*(obj.prev_pos(missing_markers,:))' + T;
            end
            centroids = obj.this_pos;
        end


        function [Rot,T,theta,trans] = pose_estimation(obj,A,B)

            [Rot,T] = rigid_transform_3D(A',B');  % SE2 w.r.t previous frame
        end

        function plot(obj,thisFrame,n_markers_detected,i_frame)
            frame_num =  i_frame + obj.start_frame - 1;
            total_frame_num = obj.n_frames + obj.start_frame - 1;
            imshow(thisFrame) 
            set(gcf, 'Position',  [10, 50, 1000, 700])
            hold on
            plot(obj.prev_pos(:,1),1080-obj.prev_pos(:,2),'g*','LineWidth',0.5,'MarkerSize',2)
            caption = sprintf('%d blobs found in frame #%d 0f %d',...
                n_markers_detected, frame_num, total_frame_num);
            title(caption, 'FontSize', 20,'Interpreter','none');
            axis on;
            hold off
            pframe = getframe(gcf);
            writeVideo(obj.vwrite,pframe);

        end  


    end

end