function compute_point_cloud(imageNumber)



% This function provides the coordinates of the associated 3D scene point 
% (X; Y;Z) and the associated color channel values for any pixel in the 
% depth image. You should save your output in the output_file in the 
% format of a N x 6 matrix where N is the number of 3D points with 3 
% coordinates and 3 color channel values:
% 
% X_1,Y_1,Z_1,R_1,G_1,B_1
% X_2,Y_2,Z_2,R_2,G_2,B_2
% X_3,Y_3,Z_3,R_3,G_3,B_3
% X_4,Y_4,Z_4,R_4,G_4,B_4
% X_5,Y_5,Z_5,R_5,G_5,B_5
% X_6,Y_6,Z_6,R_6,G_6,B_6
% .
% .
% .
% .
%
% IMPORTANT:
% "Your output should be saved in MATLAB binary format (.mat)"
% You may use any of the four following inputs for this part:
% For example, if imageNumber is 1 then possible inputs you might need
% can have the following values:
% rgbImageFileName = 'rgbImage_1.jpg';
% depthImageFileName = 'depthImage_1.png';
% extrinsicFileName = 'extrinsic_1.txt
% intrinsicsFileName = 'intrinsics_1.txt'
%
% Output file name = 'pointCloudImage_1.mat'


% add the corresponding folder name to the path 
addpath(num2str(imageNumber));

% You can remove any inputs you think you might not need for this part:
rgbImageFileName = strcat('rgbImage_',num2str(imageNumber),'.jpg');
depthImageFileName = strcat('depthImage_',num2str(imageNumber),'.png');
extrinsicFileName = strcat('extrinsic_',num2str(imageNumber),'.txt');
intrinsicsFileName = strcat('intrinsics_',num2str(imageNumber),'.txt');

rgbImage = imread(rgbImageFileName);
depthImage = imread(depthImageFileName);
extrinsic_matrix = load(extrinsicFileName);
intrinsics_matrix = load(intrinsicsFileName);


%%% YOUR IMPLEMENTATION GOES HERE:
%the projection is fx/z and fy/z, as the z dimention is shown as the depth
%so we can easily derivate the x and y coordinate in the following process.


%% get the coordinate of the pixels
%     |fx 0 cx|
% k = |0 fy cy|
%     |0  0  1|
%% Construct the principal point:
center_x = intrinsics_matrix(1,3);
center_y = intrinsics_matrix(2,3);

%depthImage = double(depthImage .* -1);
focal_length = intrinsics_matrix(1,1);

output = zeros([size(depthImage) 3]);

width = size(depthImage,2);
height = size(depthImage,1);

for indexW = 1:width
    for indexH = 1:height
        % copy z value
        z = depthImage(indexH,indexW);
        z = double(z);
        output(indexH,indexW,3) = -z;
        % calculate x value
        m = double(depthImage(indexH,indexW) / focal_length);
        r = indexW - center_x;
        x = double(m(1) * r(1));
        output(indexH,indexW,1) = x;
        % calculate y value
        u = double(depthImage(indexH,indexW) / focal_length);
        v = indexH - center_y;
        y = double(u(1) * v(1));
        output(indexH,indexW,2) = -y;
    end
end
X = output(:,:,1);
Y = output(:,:,2);
Z = output(:,:,3);
X = reshape(X, size(depthImage,1),[]);
Y = reshape(Y, size(depthImage,1),[]);
Z = reshape(Z, size(depthImage,1),[]);

pcshow([X(:),Y(:),Z(:)],reshape(rgbImage,[],3));
title('3D point cloud with RGB');
xlabel('X');
ylabel('Y');
zlabel('Z');

rgb_matrix = double(reshape(rgbImage,(size(rgbImage,1)*size(rgbImage,2)),3));
threed_point = reshape(output,(size(depthImage,1) * size(depthImage,2)),3);
res = horzcat(threed_point, rgb_matrix);

rot_trans_matrix = extrinsic_matrix(:,1:3); % obtaining rotation_translation matrix
for index=1:size(res,1)
    camera_cord = res(index,1:3)';
    world_cord = rot_trans_matrix \ camera_cord; % translate cordinates
    res(index,1:3) = world_cord';
end






%To save your ouptut use the following file name:
outputFileName = strcat('pointCloudImage_',num2str(imageNumber),'.mat');

save(outputFileName, 'res')



end
