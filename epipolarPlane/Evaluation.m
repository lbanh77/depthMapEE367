%% filename
target = 'cotton'; casenum = 2;
load ([target, '_', num2str(casenum)]);
load (['../Data/benchmark/training/', target, '/LF.mat'])
%% psnr
[~,~,Z] = getPointcloud(LF,'disp',Depth);
gt = parsePfm(['../Data/benchmark/training/', target, '/gt_depth_lowres.pfm']);
psnr_depth = psnr(Z, gt*1000, max(max(Z))); % depth
psnr_disp = psnr(Depth, LF.disp_lowres, max(max(Depth))); % disparity


%% gen depth images
figure;imshow(Z,[3.6191e+03 5.2207e+03]); colormap(flipud(parula)); colorbar;
% dino [6.0033e+03 8.1116e+03]
% boxes [971.7508 1.6157e+03]
% cotton [3.6191e+03 5.2207e+03]

%% diff image
diff_img = (Z - gt).^2;
figure; imshow(diff_img, []); colorbar;
%% MSE
mse_disp = mean((Depth - LF.disp_lowres).^2, 'all');

%% Mask
D = D_all{1};
mask1 = (D~=100);
[~,~,Zmask] = getPointcloud(LF,'disp',D);
Zmask = Zmask.*mask1;
gtmask = gt.*mask1;

figure;imshow((Zmask),[3.6191e+03 5.2207e+03]); colormap(flipud(parula)); colorbar;
psnr_mask = 10*log10(max(max(Zmask))^2 / sum((Zmask-gtmask*1000).^2, 'all')*sum(mask1,'all')); % depth
diff_img = (Zmask - gtmask).^2;
figure; imshow(diff_img, []); colorbar;






