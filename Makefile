CXX = g++
CXXFLAGS = -g -Wall -O3 -I /usr/include/eigen3/

TARGET = a.out

all: clearTerm build run

clearTerm:
		clear

build:	raytracer.cpp raytracer.h objects.cpp objects.h
		$(CXX) $(CXXFLAGS) raytracer.cpp objects.cpp

run: 
		./$(TARGET) teapot-3.nff

clean:
		$(RM) $(TARGET)