function [avg_im, test_im] = compute_averageIm(folder, img_fmt, num)

curr = cd;
cd(folder);

files = dir(['*.' img_fmt]);

im = imread(files(1).name);
if size(im,3) > 1
    avg_im = zeros(size(im,1), size(im,2), 3);
else
    avg_im = zeros(size(im,1), size(im,2));
end

avg_im = double(avg_im);


for i = 1:num
    fprintf('.');
    
    im = imread(files(i).name);
    
    if ~isa(im, 'double')
        im = im2double(im);
    end
    
    avg_im = avg_im + im;
end
fprintf('\nComputing average image...');

avg_im = avg_im ./ num;

if nargout > 1
    r = round(1 + (100-1).*rand);
    fprintf('\nReading image no. %d', r);
    test_im = imread(files(r).name);
    
    if ~isa(test_im, 'double')
        test_im = im2double(test_im);
    end
end

fprintf('\nDONE!\n');

cd(curr);