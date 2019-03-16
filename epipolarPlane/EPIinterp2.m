tic
%% load data
target = 'truck'; casenum = 1;
% load (['../Data/benchmark/training/', target, '/LF.mat'])
tmpLF1 = squeeze(LF.LF(:,:,:,:,1));
tmpLF2 = squeeze(LF.LF(:,:,:,:,2));
tmpLF3 = squeeze(LF.LF(:,:,:,:,3));
%% parameters
win_length = [3 7];
meanshiftIterationNum = 10;
disp_step = [11 11 11 11 11 11 11];
min_bound = 0.1;
kernel = fspecial('gaussian', [5,5], sqrt(0.5));
constant = 100;
maxDownsample = 7;
% min_disp = LF.parameters.meta.disp_min;
% max_disp = LF.parameters.meta.disp_max;
% num_cam_y = LF.parameters.extrinsics.num_cams_y;
% num_cam_x = LF.parameters.extrinsics.num_cams_x;
min_disp = -2;
max_disp = 2;
num_cam_y = 17;
num_cam_x = 17;
totalLF = num_cam_x*num_cam_y;
t_hat = (num_cam_y+1)/2;
s_hat = (num_cam_x+1)/2;
% threshold = repmat([0.2], [1 maxDownsample]);
threshold = [0.05 0.1 0.1 0.1 0.1 0.1 0.1];
h = 0.1;
% epsilon = [0.04, 0.04, 0.02, 0.02, 0.01, 0.01, 0 ].*0.75;
epsilon = repmat([0.03], [1 maxDownsample]);
%% initialization
% rgb2onedim = @(x) sqrt(x(:,:,1).^2 + x(:,:,2).^2 + x(:,:,3).^2)./sqrt(3);
D_all = {};
S_all = {};
Ce_all = {};
[s, t] = meshgrid(1:num_cam_x, 1:num_cam_y);
s = s(:); t = t(:);
I = (squeeze(LF.LF((num_cam_y+1)/2, (num_cam_x+1)/2, :, :, :))); % 512,512,3
depthbound = ones([size(I, 1), size(I, 2), 2]).*reshape([min_disp, max_disp]', [1 1 2]);

%%
for downCount = 1:maxDownsample
    %% LPfilter and downsample
    if downCount ~= 1
        for c = 1:3
            I(:,:,c) = conv2(I(:,:,c), kernel, 'same');
        end
        I = imresize(I, 0.5, 'bilinear');
        depthbound = imresize(depthbound, 0.5, 'nearest')/2;

        [tmpLF1, tmpLF2, tmpLF3] = resizeLF(tmpLF1, tmpLF2, tmpLF3);
    end
    %% Initialize
    Ce = zeros([size(I, 1), size(I, 2)]); %[512,512]
    D = zeros(size(Ce));
    S = zeros([ size(I, 1), size(I, 2), disp_step(downCount)]);
    [U, V] = meshgrid(1:size(I, 2), (1:size(I,1))');
    %% compute Ce, Me
    if downCount~=maxDownsample
        Ce = ComputeCe_ver3(I, win_length);
    else
        Ce(:,:,:) = 1;
    end
    Ce_all{downCount} = Ce;
    %% hypothesis d, 
    db_step = (depthbound(:,:,2) - depthbound(:,:,1))/(disp_step(downCount) - 1);
    ds = zeros([size(I, 1), size(I, 2), disp_step(downCount)]);
    ds(:,:,1) = depthbound(:,:,1); ds(:,:,end) = depthbound(:,:,2);
    for i = 2:disp_step(downCount)-1
        ds(:,:,i) = ds(:,:,i - 1) + db_step;
    end
    %% extract R, compute S
%     R_all = zeros([disp_step(downCount),size(I,1), size(I,2), 3, totalLF]); % [21 512 512 3 81]
    for d_num = 1:disp_step(downCount)
        d_num
        R = zeros([size(I, 1), size(I, 2), 3, totalLF]); % [512 512 3 81]
        d = squeeze(ds(:,:,d_num)); %[512 512]
        tmpCount = 0;
        for s = 1:num_cam_x
            for t = 1:num_cam_y
                tmpCount = tmpCount + 1;
                V_new = round(V + (t_hat - t).*d); % [512 512]
                U_new = round(U + (s_hat - s).*d); % [512 512]
                R(:,:,1,tmpCount) = interp2(U,V,squeeze(tmpLF1(t,s,:,:)),U_new,V_new);
                R(:,:,2,tmpCount) = interp2(U,V,squeeze(tmpLF2(t,s,:,:)),U_new,V_new);
                R(:,:,3,tmpCount) = interp2(U,V,squeeze(tmpLF3(t,s,:,:)),U_new,V_new);
            end
        end
        r_bar = I; % [512 512 3]
        Rnorm = squeeze(sqrt(R(:,:,1,:).^2+R(:,:,2,:).^2+R(:,:,3,:).^2)); % [512 512 81]
        for it = 1:meanshiftIterationNum
            R_dis = R - r_bar; % [512 512 3 81]
            R_dis = (sqrt(R_dis(:,:,1,:).^2+R_dis(:,:,2,:).^2+R_dis(:,:,3,:).^2)); % [512 512 1 81]
            R_dis(R_dis > h) = h; 
            K = (1 - ((R_dis./h).^2)); % [512 512 1 81]
            r_bar = sum(R.*K,4)./ sum(K,4); 
        end
        S(:,:,d_num) = sum(K,4) ./sum(Rnorm,3);
%         R_all(d_num,:,:,:,:) = R;
    end
    %% compute Cd, select best d
    S(isnan(S)) = 0;
    [Smax, Dnum] = max(S,[],3); % S:[512 512 41]
    S_bar = mean(S,3);
    
    for v = 1:size(I,1)
        d_ind = sub2ind(size(ds), repmat(v, [1 size(I,2)]), 1:size(I,2), Dnum(v,:));
        D(v,:) = ds(d_ind);
    end
    
    if downCount < 2
        Cd = Ce.* (Smax-S_bar); % [512 512]
    else
        Cd = Ce;
    end
   
    if downCount ~= maxDownsample
        MaskCd = Cd > epsilon(downCount);
        D(~MaskCd) = constant;
    end
    
    %% bilateral filter 
    % TODO
    D = medfilt2((D));
    %% save
    D_all{downCount} = D;
    S_all{downCount} = S;
    %% update depthbound
    depthbound = extractDepthbound(D, min_disp, max_disp, min_bound, constant);
   
   
end

%% upsampling and medfilt
Depth = upsampling(D_all, constant, 0);
Depth = medfilt2(Depth, [3,3]);
figure; imshow(Depth,[]); 
toc
%% psnr
% [~,~,Z] = getPointcloud(LF,'disp',Depth);
% gt = parsePfm(['../Data/benchmark/training/', target, '/gt_depth_lowres.pfm']);
% psnr_depth = psnr(Z, gt*1000, max(max(Z))); % depth
% psnr_disp = psnr(Depth, LF.disp_lowres, max(max(Depth))); % disparity
%% testing
% for i = 1:maxDownsample
% test = (D_all{i});
% figure; imshow(test,[min_disp,max_disp]);
% end
%% save
save ([target,'_',num2str(casenum), '.mat'], ['Depth'], ['D_all'], ['Ce_all'], ['S_all']);
