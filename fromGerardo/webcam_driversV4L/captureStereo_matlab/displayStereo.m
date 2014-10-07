function displayStereo(width, height)

close all;
fileName = 'im';
reply = 'y';
figure(1);
figure(2);
while strcmp('y', reply)
    
	unix(['./captureStereo -o ' fileName ' --focus_left 0 --focus_right 0 --YUV -W ' num2str(width) ' -H ' num2str(height)]);
    
    imL = readYUVbin([fileName 'L.yuv'], width, height);
    imR = readYUVbin([fileName 'R.yuv'], width, height);
    
    imLg = rgb2gray(imL);
    imRg = rgb2gray(imR);
    
    anaglyph = cat(3,imLg,imRg,imRg);
    stereo = appendimages(imL, imR);
    
    figure(1); imshow(stereo,'Border', 'tight');
    figure(2); imagesc(anaglyph);
    
    reply = input('Next? y/n [y]: ', 's');
	if isempty(reply)
		reply = 'y';
	end
    
end


function im = appendimages(image1, image2)

% Select the image with the fewest rows and fill in enough empty rows
%   to make it the same height as the other image.
rows1 = size(image1,1);
rows2 = size(image2,1);

if (rows1 < rows2)
    image1(rows2,1) = 0;
else
    image2(rows1,1) = 0;
end

% Now append both images side-by-side.
im = [image1 image2];

