function psnr = compute_psnr(im1, im2, maxi)
% maxi = 255 for images in the interval [0..255]

if im1 == im2
    error('Images are identical: PSNR has infinite value');
end

n = numel(im1);
error_dif = im1 - im2;
mse = (1/n) * (sum(error_dif(:).^2));
psnr = 20 * log10(maxi/sqrt(mse));
fprintf('PSNR = +%5.2f dB\n', psnr);