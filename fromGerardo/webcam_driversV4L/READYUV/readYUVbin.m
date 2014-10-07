function [rgb, yuv] = readYUVbin(fileName, width, height)

% Reads YUV binary file from webcams
% INPUTS:
%   fileName -> Name of the file
%   width -> Image width
%   height -> Image height
% OUTPUTS:
%   yuv -> Y, U, V components as columns (Mx3)
%   rgb -> RGB images (MxNx3)

fid = fopen(fileName, 'rb');
m = fread(fid, inf, 'uchar');
fclose(fid);
m = m(1:end-1);

Y = m(1:2:end);
tU = m(2:4:end);
tV = m(4:4:end);

U = zeros(length(Y),1);
V = U;

U(1:2:end) = tU;
U(2:2:end) = tU;
V(1:2:end) = tV;
V(2:2:end) = tV;

yuv = [Y U V]./255;

trgb = ycbcr2rgb(yuv);
rgb(:,:,1) = reshape(trgb(:,1), width, height)';
rgb(:,:,2) = reshape(trgb(:,2), width, height)';
rgb(:,:,3) = reshape(trgb(:,3), width, height)';



