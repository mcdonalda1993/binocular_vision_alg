all:
	g++ captureStereo.cpp -o captureStereo -I.. -L.. -llogitech -ljpeg -lwebcam `pkg-config --cflags opencv` `pkg-config --libs opencv`

clean:
	rm -f *.o
	rm -f captureStereo
