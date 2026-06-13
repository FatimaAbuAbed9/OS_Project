CC     = gcc
CFLAGS = -Wall -O2 -std=c11
RAYLIB = -lraylib -lm

.PHONY: all milestone1 milestone2 milestone3 milestone4 milestone4-headless milestone5 milestone5-headless milestone6 milestone6-headless test clean

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

# Milestone 4: multiple processes + a parent process (GUI, for grading).
# Run with: ./sim <file_name>
milestone4: sim4.c
	$(CC) $(CFLAGS) -o sim sim4.c $(RAYLIB)

# Headless build of milestone 4 used by the automated tests.
# No raylib and no display required; produces ./sim_test.
milestone4-headless: sim4.c
	$(CC) $(CFLAGS) -DHEADLESS -o sim_test sim4.c -lm

# Milestone 5: children compute own routes, report position via pipe (GUI).
milestone5: sim5.c
	$(CC) $(CFLAGS) -o sim5 sim5.c $(RAYLIB)

# Headless build of milestone 5 (no raylib, no display required).
milestone5-headless: sim5.c
	$(CC) $(CFLAGS) -DHEADLESS -o sim5_test sim5.c -lm

# Milestone 6: node mutual exclusion via POSIX semaphores in shared memory.
# Requires libpthread for sem_init / sem_wait / sem_post.
milestone6: sim6.c
	$(CC) $(CFLAGS) -o sim6 sim6.c $(RAYLIB) -lpthread

# Headless build of milestone 6.
milestone6-headless: sim6.c
	$(CC) $(CFLAGS) -DHEADLESS -o sim6_test sim6.c -lm -lpthread

# Build the headless binary and run the milestone-4 test suite.
test: milestone4-headless
	bash tests/run_tests.sh

clean:
	rm -f dijkstra sim sim_test sim5 sim5_test sim6 sim6_test *.o
