function PSNR = calculatePSNR(I_original, I_restored)

MSE = mean((I_original - I_restored).^2, 'all');

PSNR = 10*log10((max(I_original(:)))^2/MSE);
end