%% ECE 4501 - Digital Image Processing
% Experiment 3 - Motion Tracking
clear all; close all;

sizes = [360,640];               % Frame size
bt_space = [105, 251];            % Location of image template at [95, 255]
box = [120, 38];                 % Size of image template [114, 29]
framesize = 142;                 % Number of frames (142)
framestart = 0;                 % Start frame # of stream
% Jump is at frame 60
track = zeros(framesize, 2);     % Vector Array of tracker coordinates
track_t = 40;                    % Tracking threshold, if too far away track(i) = track(i-1)
% Build stream of all frames
cd '~/Documents/UVA 2018/ECE 4501/Experiment 3/Biker/img';
stream = zeros(framesize,sizes(1),sizes(2));
for i = 1:framesize
    string = strcat(num2str(i+framestart,'%04.f'),'.jpg'); % String formatter 'xxxx.jpg'
    frame = imread(string);
    stream(i,:,:) = frame(:,:,2); % Only take green channel, his shirt is green
end
frame1 = imread('0001.jpg');
frame1 = mat2gray(squeeze(frame1(:,:,2)));

cd '~/Documents/UVA 2018/ECE 4501/Experiment 3';

% Build template to match
Bt = zeros(box(1),box(2));
%framefilter = gb2d(zeros(sizes(1),sizes(2)),4,1.1,pi/16);
framefilter = gb2d(zeros(sizes(1),sizes(2)),3.5,.8,pi/4);
%frame1filter = gb2d(Bt,4,.7,pi/16);
%frame1 = mat2gray(squeeze(stream(1,:,:)));
%frame1 = imfilter(frame1,frame1filter,'symmetric');
Bt = frame1(bt_space(1):bt_space(1)+size(Bt,1)-1,bt_space(2):bt_space(2)+size(Bt,2)-1);

%%
newmatch = 0;
for f = 1:framesize % For every frame
    frame = mat2gray(squeeze(stream(f,:,:)));
    %frame = imfilter(frame,framefilter,'symmetric');
    olddiff = 100000; % Set diff to an impossibly high value
    for r = 1:size(frame,1)-size(Bt,1) % For every row
        for c = 1:size(frame,2)-size(Bt,2) % For every col
            Btrc = frame(r:r+size(Bt,1)-1,c:c+size(Bt,2)-1); % Create iterative frame to compare
            diff = sum(sum(abs(Btrc-Bt))); % Difference template and iterative image box
            if diff < olddiff
                if f > 1
                    % Find difference between new frame and old frame track
                    if sqrt((track(f,1)-track(f-1,1)).^2 + (track(f,2)-track(f-1,2)).^2) < track_t && f > 1
                        olddiff = diff;
                        Btmatch = Btrc;
                        track(f,1) = r;
                        track(f,2) = c;
                    % If too far then use old track position
                    else
                        track(f,1) = track(f-1,1);
                        track(f,2) = track(f-1,2);
                    end
                else
                    olddiff = diff;
                    Btmatch = Btrc;
                    track(f,1) = r;
                    track(f,2) = c;
                end
            end
        end
    end
    if f > 1 && newmatch == 0
        % Again difference old and new frames - boolean created to ensure
        % flip-flopping of matches doesn't occur
        if sqrt((track(f,1)-track(f-1,1)).^2 + (track(f,2)-track(f-1,2)).^2) > track_t
            Bt = frame(track(f-1,1):track(f-1,1)+size(Bt,1)-1,track(f-1,2):track(f-1,2)+size(Bt,2)-1);
            track(f,1) = track(f-1,1);
            track(f,2) = track(f-1,2);
            newmatch = 1;
            %Bt = Btmatch; % Frame match found in last sequence used to generate new comparison frame
            %track(f,1) = track(f-1,1);
            %track(f,2) = track(f-1,2);
        else
            newmatch = 0;
        end
    end
end

%% Finally do another frame diff to remove any oddities in tracker diff.
for f = 2:framesize
    if sqrt((track(f,1)-track(f-1,1)).^2 + (track(f,2)-track(f-1,2)).^2) > track_t*2
        track(f,1) = track(f-1,1);
        track(f,2) = track(f-1,2);
    end
end
%% Tracker Visualization
% Add box around template to visualize tracker
for f = 1:framesize
    stream(f,track(f,1):track(f,1)+box(1),[track(f,2) track(f,2)+box(2)]) = 255;
    stream(f,[track(f,1) track(f,1)+box(1)],track(f,2):track(f,2)+box(2)) = 255;
end

% Create AVI video and play using MATLAB Video Viewer
v = VideoWriter('Biker.avi');
open(v);
for i = 1:size(stream, 1)
    imgray = squeeze(stream(i,:,:))./max(max(stream(i,:,:)));
    vid(i) = im2frame(cat(3, imgray, imgray, imgray));
    writeVideo(v,vid(i));
end
close(v);
implay(vid);

%% Groundtruth Error Comparison
cd '~/Documents/UVA 2018/ECE 4501/Experiment 3/Biker/';
groundtruth = csvread('groundtruth_rect.txt');
error_rows = abs(track(:,2)-groundtruth(:,1));
error_cols = abs(track(:,1)-groundtruth(:,2));
plot_x = 1:framesize;
plot(plot_x,error_rows,plot_x,error_cols);
legend({'Row Error','Column Error'});
xlabel('Frame (#)');
ylabel('Error = Track(x/y) - Truth(x,y)');