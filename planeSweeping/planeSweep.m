close all;
addpath(genpath('../matlab-tools')); 
%% heidelberg data set
%%%% 9x9 light field, 512 x 512 images
%%%% uncomment the rest of this section if testing these images
numLF = 9;
h = 512; w = 512;
folder = '../Data/benchmark/training/cotton';
% folder = '../Data/benchmark/training/dino';
% folder = '../Data/benchmark/training/boxes';
convertBlenderTo5D(folder);
load([folder '/LF.mat']);
lfStruct = LF; 
LF = LF.LF;

%% lytro dataset
%%%% 17x17 light field 960 x 1280 images (takes some time bc of image size)
%%%% uncomment the rest of this section if testing these images
% numLF = 17;
% h = 960/2; w = 1280/2;
% LF = zeros(numLF, numLF, h, w, 3);
% folder = 'rectified';
% files = dir([folder '\*.png']);
% fNames = {files.name};
% 
% for f = 1:length(fNames)
%     img = imresize(im2double(imread([folder '/' fNames{f}])), 1/2);
%     splitFName = split(fNames{f}, '_');
%     u = str2double(splitFName{2}) + 1; % vertical direction in LF
%     v = str2double(splitFName{3}) + 1; % horizontal direction in LF
%     LF(u, v, :, :, :) = img;
% end

%%
nPoints = 11;
% mse = zeros(length(sweepPlanes),1);
% runTimes = zeros(length(sweepPlanes), 1);

dRange = linspace(-2,2,nPoints);
nPlanes = length(dRange);
S = zeros(h, w, nPlanes);
centerLF = ceil(numLF/2);
m = zeros(h, w, 3, nPlanes);
idx = 1;

tic
for d = dRange
    temp = [];
    
    for u = centerLF - 3:centerLF +3 
        for v = centerLF - 3:centerLF +3 
            img = squeeze(LF(u, v, :, :, :));
            img = imtranslate(img, [-(centerLF-v)*d -(centerLF-u)*d]);
            temp = cat(4, temp, img);
        end
    end
    
    m(:, :, :, idx) = mean(temp, 4);
    % try different cost functions: box, bilatfilt
    S(:, :, idx) = imboxfilt(squeeze(mean(sqrt(sum((temp - m(:, :, :, idx)).^2, 3)), 4)));
    idx = idx + 1;
end
toc

[confidence, disparity] = min(S, [], 3);
disparity = disparity.*4./nPlanes - 2;

%%
groundTruth = parsePfm([folder '/gt_depth_lowRes.pfm']);
figure; imshow(groundTruth, []); colorbar; colormap(flipud(parula));

%% Only use this section if running Heidelberg dataset
[~, ~, depth] = getPointcloud(lfStruct, 'disp', disparity);
Psnr = calculatePSNR(groundTruth.*1000, depth)

% display limits 
if strcmp(folder,'dino' ) == 1
    limit = [6.0033e+03 8.1116e+03]; 
elseif strcmp(folder,'boxes' ) == 1
    limit = [971.7508 1.6157e+03]; 
elseif strcmp(folder,'cotton' ) == 1
    limit = [3.6191e+03 5.2207e+03]; 
else 
    limit = []; 
end 
figure; imshow(depth, limit); colorbar; colormap(flipud(parula));