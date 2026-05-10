CC     = gcc
CFLAGS = -Wall -O2 -std=c11
RAYLIB = -lraylib -lm

.PHONY: all milestone1 milestone2 milestone3 clean

all: milestone1 milestone3

milestone1: dijkstra
dijkstra: main.c
	$(CC) $(CFLAGS) -o dijkstra main.c

# Milestone 2 (static graph display) and milestone 3 (animation)
# share the same binary `sim`, since milestone 3 builds on milestone 2.
milestone2: sim
milestone3: sim
sim: sim.c
	$(CC) $(CFLAGS) -o sim sim.c $(RAYLIB)

clean:
	rm -f dijkstra sim *.o
