classdef OfflineTracking
    properties
        number_of_markers;
        centroids;
        PrevPt;
        P0;
        CurrPt;
        cent;
        tracking_data_centroid; % Added
        
        vread;
        numberOfFrames;
		% start_frame;
        vwrite;
        
        overlay_image;
        overlay;
    end
    methods
        function obj = OfflineTracking(params)
            obj.number_of_markers = params.number_of_markers;
            obj.PrevPt = [];
            obj.P0 = [];
            obj.CurrPt = [];
            obj.cent = [];
            obj.tracking_data_centroid = [];  % Added
            obj.vread = params.vread;
            obj.numberOfFrames = obj.vread.NumberOfFrame;
            obj.vwrite = params.vwrite;
            obj.centroids = zeros(obj.numberOfFrames,3*obj.number_of_markers);
            % obj.overlay_image = params.overlay_img_cut;
            obj.overlay = params.overlay;
			% obj.start_frame = params.start_frame;
        end
					
        function start_frame_robot = find_start_frame(obj)
            ii = 1;
            while(ii)
    			thisFrame = read(obj.vread,ii);
    			newim = createMaskGreen(thisFrame);
    			newim = bwareaopen(newim,40);
    			newim = imfill(newim, 'holes');

    			[labeledImage, numberOfRegions] = bwlabel(newim);

                if numberOfRegions == 3
                    ii = ii+1;
                end

                if numberOfRegions < 3
                    start_frame_robot = ii;
                    ii = 0;
                end
            end
        end

			
		% 	count = 0;
		% 	cent = zeros(numberOfRegions,2);
        % 
		% 	stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area','EquivDiameter');
		% 	for rb = 1:numberOfRegions
			% 	count = count + 1;
			% 	cent(count,:) = stats(rb).Centroid;
			% 	cent(count,2) = 1080 - obj.cent(count,2) ;  % Correction for y-axis.
		% 	end
        % 
		% 	if ii == 1
		% 	centroid(ii,:) = [mean(cent(:,1)) mean(cent(:,2))];
		% 	end
        % 
		% 	if ii ~= 1
		% 	centroid(ii,:) = [mean(cent(:,1)) mean(cent(:,2))];
		% 	X = [centroid(ii-1,:);centroid(ii,:)];
        %     d(ii) = pdist(X,'euclidean');
			% 	if d > 1000
				% 	start_frame = ii-1;
				% 	ii = 0;
			% 	end
		% 	end
		% 	end
		% end
			
		
        function [tracking_data] = tracking(obj)
            for k = 1:obj.numberOfFrames
                thisFrame = read(obj.vread,k);
                newim = createMaskBlue(thisFrame);
                newim = bwareaopen(newim,40);
                newim = imfill(newim, 'holes');

                [labeledImage, numberOfRegions] = bwlabel(newim);

                count = 0;
                obj.cent = zeros(numberOfRegions,2);

                stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area','EquivDiameter');
                 for rb = 1:numberOfRegions
%                      aa = stats(rb).Centroid;
%                      if aa(1) > 10
                     count = count + 1;
                     obj.cent(count,:) = stats(rb).Centroid;
                     obj.cent(count,2) = 1080 - obj.cent(count,2) ;  % Correction for y-axis.
%                      end
                 end

                 obj.tracking_data_centroid(k,:) = [mean(obj.cent(:,1)) 1080-mean(obj.cent(:,2))];   % Added ANM (Purely for plotting purpose)
%                  tc(k,:) = obj.tracking_data_centroid(k,:);     
             
                  zc = zeros(size(obj.cent,1),1);
                  obj.cent = [obj.cent,zc];
                  if k == 1
                    obj.P0 = obj.cent;
                    obj.PrevPt = obj.cent;
                    obj.centroids = data_logging(obj,k);
                    plot(obj,thisFrame,count,k,newim);
                  end

                  if k ~= 1

                     obj.CurrPt = nearest_neighbor(obj,count);
%                      obj.CurrPt = obj.cent;

                     [Rot,T] = pose_estimation(obj,obj.CurrPt,obj.PrevPt,k);
                     theta(k,:) = reshape(Rot,[1,9]);
                     trans(k,:) = T';

                     [Rot,T] = pose_estimation(obj,obj.P0,obj.CurrPt,k);
                     theta_G(k,:) = reshape(Rot,[1,9]);
                     trans_G(k,:) = T';

                     obj.PrevPt = obj.CurrPt;
                     obj.centroids = data_logging(obj,k);

                     plot(obj,thisFrame,count,k,newim);

                  end

            end

            tracking_data = cat(2,obj.centroids,theta,trans,theta_G,trans_G);
            close(obj.vwrite);
        end

        function centroids = nearest_neighbor(obj,count)      %Change the name of the centroid ->
            
            obj.CurrPt = zeros(obj.number_of_markers,3);
            if(count > obj.number_of_markers)
                for i = 1:obj.number_of_markers
                    for j = 1:count
                        X = [obj.PrevPt(i,:);obj.cent(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);  
                    if(dmin < 25)
                        obj.CurrPt(i,:) = obj.cent(ind,:);
                    end
                end
            end
            if(count <= obj.number_of_markers)
                for i = 1:count
                    for j = 1:obj.number_of_markers
                        X = [obj.cent(i,:);obj.PrevPt(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);
                    if(dmin < 25)
                        obj.CurrPt(ind,:) = obj.cent(i,:);
                    end
                end
            end
            
            clear d;
            TF = obj.CurrPt(:,1);  % Writing the 1st column of resrvd
            index = find(TF == 0);  % Finding those rows which is empty
            val = isempty(index);  % Checking whether the index is empty  
            
            if(val == 0)
                centroids = occlusion(obj,index);   %Change the name of the centroid ->
            end
            if(val~=0)
                centroids = obj.CurrPt;       %Change the name of the centroid ->
            end

        end 

        function centroids = occlusion(obj,index)   %Change the name of the centroid ->
            newPrevPt = obj.PrevPt;
            newP0 = obj.P0; 
            newPrevPt(index(1:size(index,1)),:) = 0;
            newP0(index(1:size(index,1)),:) = 0;
%             [Rot,T,~,~] = pose_estimation(newPrevPt,obj.CurrPt); % SE2 w.r.t previous frame
            [Rot,T] = pose_estimation(obj,newPrevPt,obj.CurrPt); % SE2 w.r.t previous frame
            for gg = 1:size(index,1)
                newPt = Rot*(obj.PrevPt(index(gg),:))' + T;
                obj.CurrPt(index(gg),:) = newPt;
            end
            centroids = obj.CurrPt;
        end

        function centroids = data_logging(obj,k)

            for i = 1:obj.number_of_markers
                obj.centroids(k,(3*i)-2:(3*i)) = obj.PrevPt(i,:);
                centroids = obj.centroids;
            end

        end

        function [Rot,T,theta,trans] = pose_estimation(obj,A,B,k)

            [Rot,T] = rigid_transform_3D(A',B');  % SE2 w.r.t previous frame
%             theta(k,:) = reshape(Rot,[1,9]);   % Changed ANM
%             trans(k,:) = T';

        end

        function plot(obj,thisFrame,count,k,newim)

            figure(10)
            imshow(thisFrame)
%             imshow(newim);
            set(gcf, 'Position',  [100, 100, 1000, 1000])
            hold on
            plot(obj.PrevPt(:,1),1080-obj.PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',2)
%             plot(obj.tracking_data_centroids(k,1),1080-obj.tracking_data_centroid(k,2),'g*','LineWidth',0.5,'MarkerSize',2);
            
            caption = sprintf('%d blobs found in frame #%d 0f %d', count, k, obj.numberOfFrames);
            title(caption, 'FontSize', 20);
            axis on;
            hold off
            pframe = getframe(gcf);
            writeVideo(obj.vwrite,pframe);

        end  
%         
%         function plot(obj,thisFrame,count,k)
% % 
%             figure(10)
%             imshow(thisFrame)
%             set(gcf, 'Position',  [100, 100, 1000, 1000])
%             hold on
% %             plot(obj.PrevPt(k,1),1080-obj.PrevPt(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
%             plot(obj.tracking_data_centroid(k,1),1080-obj.tracking_data_centroid(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
%             caption = sprintf('%d blobs found in frame #%d 0f %d', count, k, obj.numberOfFrames);
%             title(caption, 'FontSize', 20);
%             axis on;
% %             hold off
%             ovly_img_hdl = imagesc(obj.overlay_image);              % overlay (reference) image
%             set(ovly_img_hdl, 'AlphaData', 0.4);    % set overlaid image alpha
%             hold off;
%             pframe = getframe(gcf);
%             writeVideo(obj.vwrite,pframe);
% 
%         end 

    end

end