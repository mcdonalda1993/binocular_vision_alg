all:
	g++ PSNR.cpp -o PSNR -I.. -L.. -llogitech -ljpeg -lwebcam `pkg-config --cflags opencv` `pkg-config --libs opencv`

clean:
	rm -f *.o
	rm -f PSNR
