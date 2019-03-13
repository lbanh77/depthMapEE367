dataset = 'cotton';
image_sz = [512, 512];
image_u_sz = 9;
image_v_sz = 9;
lf_sz = image_u_sz * image_v_sz;

% D = 1.149999;           % Focus Plane Distance
% F = 0.1;            % Focal Length
% F_num = 100;        % F number
% v = 0.5*(F / F_num);      % Aperture Size

lf_array = zeros(image_sz(1), image_sz(2), 3, image_u_sz, image_v_sz);

for idx=1:81
    lf_array(:,:,:,mod(idx,image_u_sz)+1, ceil(idx/image_u_sz)) = im2double(imread(['../' dataset '/input_Cam' num2str(idx-1,'%.3u') '.png']));
end

tic

I_x = zeros(image_sz(1), image_sz(2), 3, image_u_sz, image_v_sz);
I_y = zeros(image_sz(1), image_sz(2), 3, image_u_sz, image_v_sz);
I_Vx = zeros(image_sz(1), image_sz(2), 3, image_u_sz, image_v_sz);
I_Vy = zeros(image_sz(1), image_sz(2), 3, image_u_sz, image_v_sz);

for idx=1:81
    I_x(:,1:image_sz(2)-1,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)) = diff(lf_array(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)), [], 2);
    I_y(1:image_sz(1)-1,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)) = diff(lf_array(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)), [], 1);  
end

for jdx=1:image_sz(1)
    for idx=1:image_sz(2)
        I_Vx(jdx, idx, :, 1:image_u_sz-1, :) = diff(lf_array(jdx,idx,:,:,:), [], 4);
        I_Vy(jdx, idx, :, :, 1:image_v_sz-1) = diff(lf_array(jdx,idx,:,:,:), [], 5); 
    end
end

h_num = zeros(image_sz(1), image_sz(2), 3);
h_den = zeros(image_sz(1), image_sz(2), 3);

for idx=1:81
    h_num = h_num + I_x(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)).*I_Vx(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)) + I_y(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)).*I_Vy(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz));
    h_den = h_den + I_x(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)).^2 + I_y(:,:,:,mod(idx,image_u_sz)+1,ceil(idx/image_u_sz)).^2;
end

h = h_num./h_den;

h(isnan(h)) = 0;

h=mean(h,3);

toc

%figure, imshow(I_x(:,:,:,2,2))
%figure, imshow(I_y(:,:,:,2,2))
%figure, imshow(I_Vx(:,:,:,2,2))
%figure, imshow(I_Vy(:,:,:,2,2))


%d = 1./((h/v)*(1/F - 1/D) + 1/D);

%d_scaled = -1 + 2.*(d - min(d, [], 'all'))./(max(d, [], 'all') - min(d, [], 'all'));

%h_scaled = (mean(h,3)-min(mean(h,3),[],'all'))./(max(mean(h,3), [], 'all') - min(mean(h,3), [], 'all'));

%imshow(h_scaled), colorbar

%imshow(lf_array(:,:,:,2))



addpath(genpath('../matlab-tools'));
addpath(['../' dataset]);
convertBlenderTo5D(['../' dataset]);
load(['../' dataset '/LF.mat']);
[X, Y, Z] = getPointcloud(LF, 'disp', h);
imshow(Z, []); colormap(flipud(parula)); colorbar;

imwrite(Z, [dataset '_adelson_depth.png']);

addpath('..');
depthGroundTruth = parsePfm(['../' dataset '/gt_depth_lowres.pfm'])*1000;
figure, imshow(depthGroundTruth, []); colormap(flipud(parula)); colorbar;

psnr(depthGroundTruth, Z);
