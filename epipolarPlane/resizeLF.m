function [tmpLF1, tmpLF2, tmpLF3] = resizeLF(tmpLF1, tmpLF2, tmpLF3)
    [y, x, v, u] = size(tmpLF1);
    tmpLF1_copy = tmpLF1;
    tmpLF2_copy = tmpLF2;
    tmpLF3_copy = tmpLF3;
    v = round(v/2);
    u = round(u/2);
    tmpLF1 = zeros([y, x, v, u]);
    tmpLF2 = zeros([y, x, v, u]);
    tmpLF3 = zeros([y, x, v, u]);
    for r = 1:y
        for c = 1:x
            tmpLF1(r,c,:, :) = imresize(squeeze(tmpLF1_copy(r,c,:, :)), 0.5, 'bilinear');
            tmpLF2(r,c,:, :) = imresize(squeeze(tmpLF2_copy(r,c,:, :)), 0.5, 'bilinear');
            tmpLF3(r,c,:, :) = imresize(squeeze(tmpLF3_copy(r,c,:, :)), 0.5, 'bilinear');
        end
    end

end