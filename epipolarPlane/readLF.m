numLF = 17;
h = 960/2; w = 1280/2;
LF.LF = zeros(numLF, numLF, h, w, 3);
folder = '../Data/rectified/';
files = dir([folder '/*.png']);
fNames = {files.name};

for f = 1:length(fNames)
    img = imresize(im2double(imread([folder '/' fNames{f}])), 1/2);
    splitFName = split(fNames{f}, '_');
    u = str2double(splitFName{2}) + 1; % vertical direction in LF
    v = str2double(splitFName{3}) + 1; % horizontal direction in LF
    LF.LF(u, v, :, :, :) = img;
end