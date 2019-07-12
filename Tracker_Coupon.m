%% ECE 4501 - Digital Image Processing
% Experiment 3 - Motion Tracking
clear all; close all;

sizes = [240,320];               % Frame size
bt_space = [64,146];             % Location of image template at 30, 56
box = [94, 54];                 % Size of image template square(36)
framesize = 327;                % Number of frames
track = zeros(framesize, 2);    % Vector Array of tracker coordinates
track_t = 8;                    % Tracking threshold, if too far away track(i) = track(i-1)
% Build stream of all frames
cd '~/Documents/UVA 2018/ECE 4501/Experiment 3/Coupon/img';
stream = zeros(framesize,sizes(1),sizes(2));
for i = 1:framesize
    string = strcat(num2str(i,'%04.f'),'.jpg'); % String formatter 'xxxx.jpg'
    stream(i,:,:) = rgb2gray(imread(string));
end

cd '~/Documents/UVA 2018/ECE 4501/Experiment 3';

% Build template to match
Bt = zeros(box(1),box(2));
%framefilter = gb2d(zeros(sizes(1),sizes(2)),4,1.1,pi/16);
framefilter = gb2d(zeros(sizes(1),sizes(2)),3.5,.8,pi/4);
%frame1filter = gb2d(Bt,4,.7,pi/16);
frame1 = mat2gray(squeeze(stream(1,:,:)));
%frame1 = imfilter(frame1,frame1filter,'symmetric');
Bt = frame1(bt_space(1):bt_space(1)+size(Bt,1)-1,bt_space(2):bt_space(2)+size(Bt,2)-1);


for f = 1:framesize % For every frame
    frame = mat2gray(squeeze(stream(f,:,:))); % Create grayscale of 'f' frame
    %frame_nofilt = frame;
    %frame = imfilter(frame,framefilter,'symmetric');
    olddiff = 10000; % Set diff to an impossibly high value
    for r = 1:size(frame,1)-size(Bt,1) % For every row
        for c = 1:size(frame,2)-size(Bt,2) % For every col
            Btrc = frame(r:r+size(Bt,1)-1,c:c+size(Bt,2)-1); % Create iterative frame to compare
            diff = sum(sum(abs(Btrc-Bt))); % Difference template and iterative image box
            if diff < olddiff
                olddiff = diff;
                Btmatch = Btrc;
                track(f,1) = r;
                track(f,2) = c;
                if f > 1
                    if sqrt((track(f,1)-track(f-1,1)).^2 + (track(f-1,2)-track(f-1,2)).^2) > track_t
                    %if(abs(sqrt(track(f,1).^2+track(f,2).^2) - sqrt(track(f-1,1).^2+track(f-1,2).^2)) > track_t)
                        track(f,1) = track(f-1,1);
                        track(f,2) = track(f-1,2);
                    end
                end
            end
        end
    end
    Bt = Btmatch; % Frame match found in last sequence used to generate new comparison frame
end
%% Tracker Visualization
% Add box around template to visualize tracker
for f = 1:framesize
    stream(f,track(f,1):track(f,1)+box(1),[track(f,2) track(f,2)+box(2)]) = 255;
    stream(f,[track(f,1) track(f,1)+box(1)],track(f,2):track(f,2)+box(2)) = 255;
end
% Create AVI video and play using MATLAB Video Viewer
v = VideoWriter('Coupon.avi');
open(v);
for i = 1:size(stream, 1)
    imgray = squeeze(stream(i,:,:))./max(max(stream(i,:,:)));
    vid(i) = im2frame(cat(3, imgray, imgray, imgray));
    writeVideo(v,vid(i));
end
close(v);
implay(vid);
implay(vid);
%% Groundtruth Error Comparison
cd '~/Documents/UVA 2018/ECE 4501/Experiment 3/Coupon/';
groundtruth = csvread('groundtruth_rect.txt');
error_rows = abs(track(:,2)-groundtruth(:,1));
error_cols = abs(track(:,1)-groundtruth(:,2));
plot_x = 1:framesize;
plot(plot_x,error_rows,plot_x,error_cols);
legend({'Row Error','Column Error'});
xlabel('Frame (#)');
ylabel('Error = Track(x/y) - Truth(x,y)');