# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g     - this flag adds debugging information to the executable file
#  -Wall  - this flag is used to turn on most compiler warnings
CFLAGS  = -g -Wall -std=c++11

main: main.o gbfhs.o
	$(CC) $(CFLAGS) -o main main.o gbfhs.o

main.o: main.cpp gbfhs.h gbfhs.cpp
	$(CC) $(CFLAGS) -c main.cpp

gbfhs.o: gbfhs.cpp gbfhs.h
	$(CC) $(CFLAGS) -c gbfhs.cpp

clean:
	rm *.o main