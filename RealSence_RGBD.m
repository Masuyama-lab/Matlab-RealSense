% ===============================================================
% Matlab code for capturing RGBD image by Intel RealSense D435i.
% (c) 2020 Naoki Masuyama
% Email: masuyama@cs.osakafu-u.ac.jp
% ===============================================================

clear

whitebg('black')

% Set filter parameter
FM = 3; % filter magunitude: FM = {1,2,3,4,5}


% Configurations for Realsense
config = realsense.config();
config.enable_stream(realsense.stream.color, 1280, 720, realsense.format.rgb8, 30);
config.enable_stream(realsense.stream.depth, 1280, 720, realsense.format.z16, 30);

% Set filters
depth_to_disparity = realsense.disparity_transform(true);
disparity_to_depth = realsense.disparity_transform(false);
dec_filter = realsense.decimation_filter();
dec_filter.set_option(realsense.option.filter_magnitude, FM);
spat_filter = realsense.spatial_filter();
spat_filter.set_option(realsense.option.filter_magnitude, FM);
spat_filter.set_option(realsense.option.filter_smooth_alpha, 0.5);
spat_filter.set_option(realsense.option.filter_smooth_delta, 20);
spat_filter.set_option(realsense.option.holes_fill, 0);
temp_filter = realsense.temporal_filter();
temp_filter.set_option(realsense.option.filter_smooth_alpha, 0.5);
temp_filter.set_option(realsense.option.filter_smooth_delta, 20);
temp_filter.set_option(realsense.option.holes_fill, 3);
hf_filter = realsense.hole_filling_filter();
hf_filter.set_option(realsense.option.holes_fill, 1);

% Set align 
align_to = realsense.stream.color;
alignedFs = realsense.align(align_to);

% Make Pipeline object to manage streaming
pipe = realsense.pipeline();

% Make pointcloud object
pcl_obj = realsense.pointcloud();

% Start streaming on an arbitrary camera with default settings
profile = pipe.start( config );



while (1)
        
% Wait for the frames
frames = pipe.wait_for_frames();

% Align the depth frames to the color stream
aligned_frames = alignedFs.process(frames);
depth_frame = aligned_frames.get_depth_frame();

% Filtering
filtered_depth_frame = depth_frame;
filtered_depth_frame = dec_filter.process(filtered_depth_frame);
filtered_depth_frame = depth_to_disparity.process(filtered_depth_frame);
filtered_depth_frame = spat_filter.process(filtered_depth_frame);
filtered_depth_frame = temp_filter.process(filtered_depth_frame);
filtered_depth_frame = disparity_to_depth.process(filtered_depth_frame);
filtered_depth_frame = hf_filter.process(filtered_depth_frame);

% Get the points cloud based on the aligned depth stream
points = pcl_obj.calculate(filtered_depth_frame);
vertices = points.get_vertices();
X = vertices(:,1,1);
Y = vertices(:,2,1);
Z = vertices(:,3,1);

% Get RGB data
color_frame = aligned_frames.get_color_frame();
pcl_obj.map_to(color_frame);
colordata = color_frame.get_data();
double_color_data = im2double(colordata);
rgb_data = [double_color_data(1:3:end)',double_color_data(2:3:end)',double_color_data(3:3:end)'];

% Resize Image
im_R = reshape(rgb_data(:,1),[1280,720]);
im_G = reshape(rgb_data(:,2),[1280,720]);
im_B = reshape(rgb_data(:,3),[1280,720]);
im_r = im_R(1:FM:end,1:FM:end);
im_g = im_G(1:FM:end,1:FM:end);
im_b = im_B(1:FM:end,1:FM:end);
if FM == 3
    im_r(428,:)=0.0; im_g(428,:)=0.0; im_b(428,:)=0.0;
end
rgb_data = [reshape(im_r,[],1), reshape(im_g,[],1), reshape(im_b,[],1)];

% Eliminate outliers
data = [X, Z, -Y, rgb_data];
data = data(data(:,1) > -1.0 & data(:,1) < 1.0,:); % X width
data = data(data(:,2) >  0.3 & data(:,2) < 2.0,:); % Z depth
data = data(data(:,3) > -1.0 & data(:,3) < 1.2,:); % Y height

% Plotting
figure(1)
% scatter3(data(:,1), data(:,2), data(:,3), 20, data(:,4:6), 'filled', 'o');
% scatter3(X,Z,-Y, 5, rgb_data, 'filled', 'o');
scatter3(data(:,1), data(:,2), data(:,3), 20,'cy', 'filled', 'o');
grid on
hold off
view([0 0]);
% xlim([-1.0 1.0]) % X width
% ylim([0.1 1.8])  % Z depth
% zlim([-1.0 1.2]) % Y height
xlim([-1.0 1.0]) % X width
ylim([0.8 1.8])  % Z depth
zlim([-0.6 0.4]) % Y height
xlabel('X');
ylabel('Z');
zlabel('Y');
drawnow

capturedRGB = zeros(size(im_r,2),size(im_r,1),3);
capturedRGB(:,:,1)=im_r';
capturedRGB(:,:,2)=im_g';
capturedRGB(:,:,3)=im_b';
figure(2);
imshow(capturedRGB);


end


% Save RGBD data
data = [X, Z, -Y, rgb_data];
save data230400 data
save capturedRGB capturedRGB



