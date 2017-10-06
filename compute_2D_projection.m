function [projectedDepthImage,projectedRGBImage]=compute_2D_projection(imageNumber,omegaT,rotationAxis,translationVector)


% This function create two images: 1) an image that contains the projected 
% depth value  (greyscale) and 2) an image that contains the projected 
% color of the 3D scene point.
%  
% INPUTS: 
% imageNumber: you may check the previous function and see how it has been
% used.
%
% omegaT is a value between 0 to pi/2 make sure not to use degrees!
%
% rotationAxis is either 'x' or 'y' or 'z' (you can use strcmp function
% to find which axis the rotation is about. 
%
% translationVector is a 3x1 vector that indicates the translation
% direction. For this assignment, it should have only 1 non-zero element,
% which defines the translation direction implicitly (i.e unlike what you 
% will do for rotation, you do not have to explicitly set a translation
% direction, the nonzero element will take care of direction and the
% translation amount).
%
% You can read the saved point cloud from the previous function and use
% that information here. You may also use any other inputs that are
% provided in the assignment description.
% 
% OUTPUTS:
% projectedDepthImage: an image that contains the projected depth
% value (greyscale)
% 
% projectedRGBImage: an image that contains the projected color of the 3D 
% scene point.





%%% YOUR IMPLEMENTATION GOES HERE:

% You have to compute the following images

% Pre-defined Step
translation_total = 2000;
steps = 120;
if(omegaT > pi/2 || omegaT < 0)
    disp('omegaT is a value between 0 to pi/2')
    return
end
omega_t = omegaT/steps;
translation_t = translation_total/steps;
% Pre-defined Translation Vector Filter Vector:

trans_x = [1,0,0]';
trans_y = [0,1,0]';
trans_z = [0,0,1]';
rotation_matrix = [0,0,0; 0,0,0;0,0,0];
rotation_matrix_step = [0,0,0; 0,0,0;0,0,0];
translation_matrix = [0,0,0];
% rotation axis detection
if(strcmp(rotationAxis,'x'))
    disp('rotation along axis: x')
    r_letter = 'X';
    rotation_matrix_step = [1,0,0;...
                            0,cos(omegaT/steps),-sin(omegaT/steps);...
                            0,sin(omegaT/steps),cos(omegaT/steps)];
elseif(strcmp(rotationAxis,'y'))
    disp('rotation along axis: y')
    r_letter = 'Y';
    rotation_matrix_step = [cos(omegaT/steps),0,sin(omegaT/steps);...
                                            0,1,0;... 
                           -sin(omegaT/steps),0,cos(omegaT/steps)];
elseif(strcmp(rotationAxis,'z'))
    disp('rotation along axis: z')
    r_letter = 'Z';
    rotation_matrix_step = [cos(omegaT/steps),-sin(omegaT/steps),0;...
                            sin(omegaT/steps), cos(omegaT/steps),0;... 
                            0,0,1];
else
    disp('Not Accepted Parameter: please input x, y, or z')
    return
end
% translation axis detection
if(translationVector*trans_x == 1)
    disp('translation along axis: x')
    t_letter = 'X';
    translation_axis = [-translation_total,0,0];
elseif(translationVector*trans_y == 1)
    disp('translation along axis: y')
    t_letter = 'Y';
    translation_axis = [0,-translation_total,0];
elseif(translationVector*trans_z == 1)
    disp('translation along axis: z')
    t_letter = 'Z';
    translation_axis = [0,0,translation_total];
else
    disp...
    ('Not Accepted Parameter: please input [0,0,1] , [0,1,0] or [1,0,0]')
    return
end





addpath(num2str(imageNumber));
rgbImageFileName = strcat('rgbImage_',num2str(imageNumber),'.jpg');
depthImageFileName = strcat('depthImage_',num2str(imageNumber),'.png');
% extrinsicFileName = strcat('extrinsic_',num2str(imageNumber),'.txt');
intrinsicsFileName = strcat('intrinsics_',num2str(imageNumber),'.txt');

rgbImage = imread(rgbImageFileName);
depthImage = imread(depthImageFileName);
% extrinsic_matrix = load(extrinsicFileName);
intrinsics_matrix = load(intrinsicsFileName);

width = size(depthImage,2);
height = size(depthImage,1);

% rot_trans_matrix = extrinsic_matrix(:,1:3);
inputFileName = strcat('pointCloudImage_',num2str(imageNumber),'.mat');
data = load(inputFileName);
res = data.res;

X = res(:,1);
Y = res(:,2);
Z = res(:,3);

to_project_x = X;
to_project_y = Y;
to_project_z = Z;
to_project_r = res(:,4);
to_project_g = res(:,5);
to_project_b = res(:,6);

% world_cords = zeros(size(res,1),3);
% for index=1:size(res,1)
%     camera_cord = res(index,1:3)';
%     world_cord = rot_trans_matrix \ camera_cord;
%     world_cords(index,:) = world_cord';
% end
% X_world = world_cords(:,1);
% Y_world = world_cords(:,2);
% Z_world = world_cords(:,3);
% 
% X_world = reshape(X_world, size(depthImage,1),[]);
% Y_world = reshape(Y_world, size(depthImage,1),[]);
% Z_world = reshape(Z_world, size(depthImage,1),[]);
% 
% X = reshape(X, size(depthImage,1),[]);
% Y = reshape(Y, size(depthImage,1),[]);
% Z = reshape(Z, size(depthImage,1),[]);


% figure;
% subplot(1,2,1);
% pcshow([X(:),Y(:),Z(:)],reshape(rgbImage,[],3));
% title('3D point cloud with RGB');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% subplot(1,2,2);
% pcshow([X_world(:),Y_world(:),Z_world(:)],reshape(rgbImage,[],3));
% title('World Cordinate');
% xlabel('X_w');
% ylabel('Y_w');
% zlabel('Z_w');

% %% produce the reprojection matrix
% % from 3d point to reprojection 2d image 
% %P2D = zeros(width*height,3);
% Reproj_DepthImage = zeros(size(depthImage));
% Reproj_RgbImage = uint8(zeros(size(rgbImage)));
% 
% for project_indice = 1:size(to_project_x,1)
%     P3D = [to_project_x(project_indice),to_project_y(project_indice),...
%         to_project_z(project_indice)]';
%     P3D_reproj = rotation_matrix * P3D + translation_total;
%     x_reproj = (P3D_reproj(1) * intrinsics_matrix(1,1) / ...
%     P3D_reproj(3)) + intrinsics_matrix(1,3) ; 
% 
%     y_reproj = (P3D_reproj(2) * intrinsics_matrix(1,1) / ...
%     P3D_reproj(3)) + intrinsics_matrix(2,3) ;
%     z_reproj = P3D_reproj(3);
%     %P2D(project_indice,3) = z_reproj;
%     x_reproj = int16(x_reproj);
%     y_reproj = int16(y_reproj);
%     %P2D(project_indice,2) = y_reproj;
%     %P2D(project_indice,1) = x_reproj;
%     if((0<x_reproj)&&(x_reproj<=width)&&(0<y_reproj)&&(y_reproj<=height))        
%         Reproj_DepthImage(y_reproj,x_reproj) = -z_reproj;
%         Reproj_RgbImage(y_reproj,x_reproj,1) = to_project_r(project_indice);
%         Reproj_RgbImage(y_reproj,x_reproj,2) = to_project_g(project_indice);
%         Reproj_RgbImage(y_reproj,x_reproj,3) = to_project_b(project_indice);
%     end
% end
%% make the movie
Reproj_DepthImage_mv = zeros(size(depthImage));
Reproj_RgbImage_mv = uint8(zeros(size(rgbImage)));
% figure;


% create video file
video_name = strcat(r_letter,t_letter,'.avi');
v = VideoWriter(video_name);
open(v);

% get the 3d point
mv_3d_x = to_project_x;
mv_3d_y = to_project_y;
mv_3d_z = to_project_z;
% loop for the steps to show the image sequences
for mv_step=1:steps
    Reproj_DepthImage_mv = zeros(size(depthImage));
    Reproj_RgbImage_mv = uint8(zeros(size(rgbImage)));
    % loop inside the image
    for rep_loop_ind = 1:size(mv_3d_x,1)
        mv_P3D = [mv_3d_x(rep_loop_ind),mv_3d_y(rep_loop_ind),...
            mv_3d_z(rep_loop_ind)]';
        mv_P3D_reproj = rotation_matrix_step * mv_P3D + ...
            (translation_axis./steps)';
        % write the rotated and translated coordinates back
        mv_3d_x(rep_loop_ind) = mv_P3D_reproj(1);
        mv_3d_y(rep_loop_ind) = mv_P3D_reproj(2);
        mv_3d_z(rep_loop_ind) = mv_P3D_reproj(3);
        % change back the 3D points into pixel coordinates
        mv_2d_x = (mv_P3D_reproj(1) * intrinsics_matrix(1,1) / ...
            mv_P3D_reproj(3)) + intrinsics_matrix(1,3);
        mv_2d_y = (mv_P3D_reproj(2) * intrinsics_matrix(1,1) / ...
            mv_P3D_reproj(3)) + intrinsics_matrix(1,3);
        mv_2d_depth = mv_P3D_reproj(3);
        mv_2d_x = int16(mv_2d_x);
        mv_2d_y = int16(mv_2d_y);
        if((0<=mv_2d_x)&&(mv_2d_x<width)&&(0<mv_2d_y)&&(mv_2d_y<=height))
            % write the depth image
            Reproj_DepthImage_mv(mv_2d_y,(width - mv_2d_x)) = abs(mv_2d_depth);
            % write the rgb image
            Reproj_RgbImage_mv(mv_2d_y,(width - mv_2d_x),1) = to_project_r(rep_loop_ind);
            Reproj_RgbImage_mv(mv_2d_y,(width - mv_2d_x),2) = to_project_g(rep_loop_ind);
            Reproj_RgbImage_mv(mv_2d_y,(width - mv_2d_x),3) = to_project_b(rep_loop_ind);
        end
    end
    Reproj_DepthImage_mv = uint16(Reproj_DepthImage_mv);
    
    subplot(1,2,1);imshow(Reproj_DepthImage_mv);title('Depth R-T Image');
    subplot(1,2,2);imshow(Reproj_RgbImage_mv);title('RGB R-T Image');
    movie_frame = getframe(gcf);
    writeVideo(v,movie_frame);
    rot_trans_movie(mv_step) = movie_frame;
    drawnow;
end

% Playback the movie
figure;
movie(rot_trans_movie,1);

write video
close(v);



%% Matrix Version
%for mv_step=1:steps
%     round_str = strcat('Round: ',mv_step)
%     rotation_matrix_step_x = rotation_matrix_step(1,:);
%     rotation_matrix_step_y = rotation_matrix_step(2,:);
%     rotation_matrix_step_z = rotation_matrix_step(3,:);
%     % rotation
%     reproj_x_mv = rotation_matrix_step_x * depth_data_mv';
%     reproj_y_mv = rotation_matrix_step_y * depth_data_mv';
%     reproj_z_mv = rotation_matrix_step_z * depth_data_mv';
%     % translation
%     rotated_data = [reproj_x_mv;reproj_y_mv;reproj_z_mv];
%     trans_tmp = translation_axis' ./ steps;
%     trans_mv = repmat(trans_tmp,[1 size(rotated_data,2)]);
%     center_x_mv = repmat(intrinsics_matrix(1,3),[1 size(rotated_data,2)]);
%     center_y_mv = repmat(intrinsics_matrix(2,3),[1 size(rotated_data,2)]);
%     reproj_3d = rotated_data + trans_mv;
%     x_reproj_mv = (reproj_3d(1,:) .* intrinsics_matrix(1,1) ./ ...
%     reproj_3d(3,:)) + center_x_mv;
%     y_reproj_mv = (reproj_3d(2,:) .* intrinsics_matrix(2,2) ./ ...
%     reproj_3d(3,:)) + center_y_mv;
%     z_reproj_mv = abs(reproj_3d(3,:)); % depth should be positive to show in image
%     
%     % reproject_matrix
%     x_reproj_mv = uint16(x_reproj_mv);
%     y_reproj_mv = uint16(y_reproj_mv);
%     z_reproj_mv = uint16(z_reproj_mv);
%     reproj_matrix_mv = [x_reproj_mv;y_reproj_mv;z_reproj_mv;rgb_data_mv'];
%     
%     
%     % flag array
% %     paint_cord_subs = intersect(find(0 < x_reproj_mv & x_reproj_mv <= width),...
% %         find(0< y_reproj_mv & y_reproj_mv <= height));
% %     paint_map = reproj_matrix_mv(:,paint_cord_subs);
% %     % paint_cord = zeros(2,size(paint_map,2));
% %     paint_cord = uint16(paint_map(1:2,:)); % x and y cord, also the sub of image matrix
% %     % paint_value = zeros(1,size(paint_map,2));
% %     paint_value = uint16(paint_map(3,:)); % z(depth)
% 
%     for sub_ind_flag = 1:size(reproj_matrix_mv,2)
%         if((0<reproj_matrix_mv(1,sub_ind_flag))&&... % x>0
%             (reproj_matrix_mv(1,sub_ind_flag)<=width)&&... % x< width
%             (0<reproj_matrix_mv(2,sub_ind_flag))&&... % y > 0
%             (reproj_matrix_mv(2,sub_ind_flag)<=height)) % y < height
%                 Reproj_DepthImage_mv(reproj_matrix_mv(2,sub_ind_flag),...
%                 reproj_matrix_mv(1,sub_ind_flag)) ... % reproj(y,x) = depth
%             = reproj_matrix_mv(3,sub_ind_flag); % assign the value of depth
%         end
%         %if(paint_cord(1,sub_ind_flag)*paint_cord(2,sub_ind_flag) <= width*height)
%             %paint_index = sub2ind(size(Reproj_DepthImage_mv),...
%             %paint_cord(2,sub_ind_flag),paint_cord(1,sub_ind_flag)); % count the index of the depth image
%         if((0<x_reproj)&&(x_reproj<width)&&(0<y_reproj)&&(y_reproj<height))        
%             Reproj_DepthImage(y_reproj,x_reproj) = -z_reproj;
%             Reproj_RgbImage(y_reproj,x_reproj,1) = to_project_r(project_indice);
%             Reproj_RgbImage(y_reproj,x_reproj,2) = to_project_g(project_indice);
%             Reproj_RgbImage(y_reproj,x_reproj,3) = to_project_b(project_indice);
%             Reproj_DepthImage_mv(paint_cord(2,sub_ind_flag),paint_cord(1,sub_ind_flag)) = paint_value(sub_ind_flag);
%             end
%         %else
%             %disp('out of boundary: ');
%             %paint_cord(:,sub_ind_flag)
%         %end
%     end
%     % Reproj_DepthImage_mv(:, 1:end) = Reproj_DepthImage_mv(:, end:-1:1);
%     imshow(Reproj_DepthImage_mv);
%     drawnow;
%     pause(0.1) %in seconds
% end



%% return the result

projectedDepthImage = Reproj_DepthImage_mv;
projectedRGBImage = Reproj_RgbImage_mv;


end
