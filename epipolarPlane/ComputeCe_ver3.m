function [Ce] = ComputeCe_new(I, win_length)
    [height, width, ~] = size(I);
    w = (win_length(2) - 1)/2;
    h = (win_length(1) - 1)/2;
    
    Ce = zeros([height, width]);
    % pad
    I = cat(2, repmat(I(:,1,:),[1,w]), I, repmat(I(:,end,:), [1,w])); % 512 x (512+2w)
    I = cat(1, repmat(I(1,:,:),[h,1]), I, repmat(I(end,:,:), [h,1])); % (512+2h) x (512+2w)

    % loop through u
    for j = w+1:w+width
        tmp = I(:,j - w: j + w, :);
        for i = -h:h
            add = sum((circshift(tmp, i) - I(:,j,:)).^2, [2, 3]);
            Ce(:,j-w) = Ce(:,j-w) + add(h+1:h+height);
        end
    end
    
    
%     Me = Ce > threshold;
%     se = strel('diamond',1);
%     Me = imdilate(imerode(Me,se), se); % [9, 512]
end