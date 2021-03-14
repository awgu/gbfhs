# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g     - this flag adds debugging information to the executable file
#  -Wall  - this flag is used to turn on most compiler warnings
CFLAGS  = -g -Wall -std=c++11

main: main.o gbfhs.o mme.o a_star.o pancake.o
	$(CC) $(CFLAGS) -o main main.o gbfhs.o mme.o a_star.o pancake.o

main.o: main.cpp gbfhs.h gbfhs.cpp mme.h mme.cpp a_star.h a_star.cpp pancake.h pancake.cpp
	$(CC) $(CFLAGS) -c main.cpp

gbfhs.o: gbfhs.cpp gbfhs.h pancake.cpp pancake.h
	$(CC) $(CFLAGS) -c gbfhs.cpp

mme.o: mme.cpp mme.h pancake.cpp pancake.h
	$(CC) $(CFLAGS) -c mme.cpp

a_star.o: a_star.cpp a_star.h pancake.cpp pancake.h
	$(CC) $(CFLAGS) -c a_star.cpp

pancake.o: pancake.cpp pancake.h
	$(CC) $(CFLAGS) -c pancake.cpp

clean:
	rm *.o main