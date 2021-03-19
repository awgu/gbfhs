# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g     - this flag adds debugging information to the executable file
#  -Wall  - this flag is used to turn on most compiler warnings
CFLAGS  = -g -Wall -std=c++11

main: main.o gbfhs.o mme.o astar.o puzzle.o
	$(CC) $(CFLAGS) -o main main.o gbfhs.o mme.o astar.o puzzle.o

main.o: main.cpp gbfhs.h gbfhs.cpp mme.h mme.cpp astar.h astar.cpp puzzle.h puzzle.cpp
	$(CC) $(CFLAGS) -c main.cpp

gbfhs.o: gbfhs.cpp gbfhs.h puzzle.cpp puzzle.h
	$(CC) $(CFLAGS) -c gbfhs.cpp

mme.o: mme.cpp mme.h puzzle.cpp puzzle.h
	$(CC) $(CFLAGS) -c mme.cpp

astar.o: astar.cpp astar.h puzzle.cpp puzzle.h
	$(CC) $(CFLAGS) -c astar.cpp

puzzle.o: puzzle.cpp puzzle.h
	$(CC) $(CFLAGS) -c puzzle.cpp

clean:
	rm *.o main