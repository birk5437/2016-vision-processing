CC = arm-frc-linux-gnueabi-g++
INC = -I../_Includes/OpenCV/2.4.10 -I../_Includes/WPILib 
IARGS = -O2 -g -Wall -c -fmessage-length=0 -std=gnu++11 -fPIC 
ODIR = ../build/src/
LINK = -L../_Libraries/softFP/lib_OpenCV -L../_Libraries/softFP/lib_WPILib -L../_Libraries/softFP/lib_cpp -Xlinker -rpath ../_Libraries/softFP/lib_cpp -Xlinker -rpath-link ../_Libraries/softFP/lib_OpenCV 
LDARGS = -lopencv_core -lopencv_highgui -lntcore -lwpi -lopencv_imgproc -lpthread -lrt 
SOURCES = main.cpp
OBJECTS = ../build/main.o
EXECUTABLE = ../build/2016-vision-processing


all: $(SOURCES) $(EXECUTABLE)


$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LINK) -o $@ $< $(LDARGS)	

$(OBJECTS): $(SOURCES)
	$(CC) $(INC) $(IARGS) -o $@ $<

clean:
	rm ../build/*
