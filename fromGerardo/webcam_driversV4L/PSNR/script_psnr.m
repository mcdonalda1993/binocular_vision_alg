clear;

folder = 'Test2';
fmt = 'tif';
numIm = 100;
init = 2;

testIm = [];
psnr_values = zeros(numIm/init, 2);

for i = init:init:numIm
    if isempty(testIm)
        [avg_im, testIm] = compute_averageIm(folder, fmt, i);
        testIm = rgb2ycbcr(testIm .* 255);
    else
        avg_im = compute_averageIm(folder, fmt, i);
    end
    
    avg_im = rgb2ycbcr(avg_im .* 255);

    psnr_values(i/init,1) = i;
    psnr_values(i/init,2) = compute_psnr(testIm(:,:,1), avg_im(:,:,1), 255);
end

