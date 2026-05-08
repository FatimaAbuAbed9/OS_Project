CC     = gcc
CFLAGS = -Wall -O2 -std=c11
RAYLIB = -lraylib -lm

.PHONY: all milestone1 milestone2 clean

all: milestone1 milestone2

milestone1: dijkstra
dijkstra: main.c
	$(CC) $(CFLAGS) -o dijkstra main.c

milestone2: sim
sim: sim.c
	$(CC) $(CFLAGS) -o sim sim.c $(RAYLIB)

clean:
	rm -f dijkstra sim *.o
