% ## Copyright (C) 2012 Gerardo Aragon-Camarasa
% ## 
% ## This program is free software; you can redistribute it and/or modify
% ## it under the terms of the GNU General Public License as published by
% ## the Free Software Foundation; either version 2 of the License, or
% ## (at your option) any later version.
% ## 
% ## This program is distributed in the hope that it will be useful,
% ## but WITHOUT ANY WARRANTY; without even the implied warranty of
% ## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% ## GNU General Public License for more details.
% ## 
% ## You should have received a copy of the GNU General Public License
% ## along with Octave; see the file COPYING.  If not, see
% ## <http://www.gnu.org/licenses/>.
% 
% ## captureFromCameras
% 
% ## Author: Gerardo Aragon-Camarasa <gerardo@shemya>
% ## Created: 2012-03-16

function captureFromCameras(fileName)

clc;

width = 2592;
height = 1944;

counter = 1;
reply = 'y';
while strcmp('y', reply)

	unix(['./captureStereo -o ' fileName num2str(counter) ' --focus_left 0 --focus_right 0 --JPEG --YUV -W ' num2str(width) ' -H ' num2str(height)]);

	unix(['gzip -c ' fileName num2str(counter) 'L.yuv > ' fileName num2str(counter) 'L.gz']);
	unix(['gzip -c ' fileName num2str(counter) 'R.yuv > ' fileName num2str(counter) 'R.gz']);

	delete([fileName num2str(counter) 'L.yuv']);
	delete([fileName num2str(counter) 'R.yuv']);

	counter = counter + 1;

	reply = input('One more? y/n [y]: ', 's');
	if isempty(reply)
		reply = 'y';
	end


end
