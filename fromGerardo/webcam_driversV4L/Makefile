all:
	g++ -Wall -c -o liblogitech.o liblogitech.cpp -I/usr/local/include/opencv -I/libwebcam-0.2.1/common/include
	ar rc liblogitech.a liblogitech.o
	rm -f liblogitech.o
	g++ -Wall testStereo.cpp -o testStereo -L. -llogitech -ljpeg -lwebcam `pkg-config --cflags opencv` `pkg-config --libs opencv`
	g++ -Wall testMono.cpp -o testMono -L. -llogitech -ljpeg -lwebcam `pkg-config --cflags opencv` `pkg-config --libs opencv`  

clean:
	rm -f *.o
	rm -f *.a
	rm -f testStereo
	rm -f testMono
