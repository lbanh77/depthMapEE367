function Depth = upsampling(D_all, constant, show)
    down_Count = size(D_all,2);
    D_down = D_all{down_Count};
    for i = 1:down_Count-1
        D_up = D_all{down_Count-i};
        Depth = zeros(size(D_up));
%         Depth = imresize(D_down, 2, 'nearest')*2;
        Depth (1:2:end, 1:2:end) = D_down; Depth (1:2:end, 2:2:end) = D_down;
        Depth (2:2:end, 2:2:end) = D_down(1:size(Depth,1)/2,:);
        Depth (2:2:end, 1:2:end) = D_down(1:size(Depth,1)/2,:);
        Depth = Depth*2;
        Depth(D_up ~= constant) = D_up(D_up ~= constant);
        D_down = Depth;
        
        if show
            figure; imshow(Depth,[]);
        end
    end
end