# OS Project — Graph Movement Simulation

Operating Systems course project: a simulation of an entity moving on a directed weighted graph along the shortest path computed by Dijkstra's algorithm.

The graph is loaded from a text file, the shortest path between a source and destination is computed, and an animated entity traverses the path in real time using the **raylib** graphics library.

---

## Build

The project uses a single `Makefile` with one target per milestone.

```bash
make milestone1   # builds the ./dijkstra binary (milestone 1)
make milestone2   # builds the ./sim binary (milestones 2 and 3)
make milestone3   # same as milestone2 — same binary, more features
make clean        # remove all compiled binaries
```

**Dependencies:** `gcc`, GNU `make`, and `raylib` (only required for milestones 2 and 3).
Install raylib on Debian/Ubuntu: `sudo apt install libraylib-dev`

---

## Run

```bash
./dijkstra <file_name>      # milestone 1: prints the shortest path to the terminal
./sim      <file_name>      # milestones 2 and 3: opens the GUI window
```

A sample input file `input.txt` is included in the repository.

---

## Input file format

```
N M             <- number of nodes, number of edges
src dst weight  <- M lines, one per directed edge
...
src dst         <- last line: the source and destination for the Dijkstra query
```

Example (`input.txt`):
```
6 8
0 1 4
0 2 2
1 3 5
2 1 1
2 3 8
3 4 2
4 5 3
2 5 10
0 5
```

Negative numbers are rejected with an error message. Node indices outside the range `0..N-1` are also rejected.

---

## Milestones

### Milestone 1 — Dijkstra on a directed weighted graph (`main.c`)

A command-line program that loads the graph from a file and prints the shortest path and total weight from source to destination.

* The graph is stored as an **adjacency list**: each node has a dynamic array of outgoing edges of the form `(dst, weight)`.
* Dijkstra's algorithm is implemented with a **binary min-heap** priority queue keyed by distance, giving `O((N + M) log N)` time complexity.
* The path is reconstructed by walking the `prev[]` array back from the destination to the source.
* If the destination is unreachable, the program prints `No path found`.
* If source equals destination, only that single node is printed.

Example run:
```
$ ./dijkstra input.txt
0 -> 2 -> 5
12
```

---

### Milestone 2 — Static graph visualization (`sim.c`)

Opens a 1000×720 window and draws the graph using **raylib**.

* Node positions are computed automatically on a circular layout (a single ring up to 8 nodes, two concentric rings beyond that), so any input file with up to 15 nodes is laid out without overlapping.
* Edges are drawn as directed arrows with the weight printed next to each one.
* Node colors:
  * **Green** — source node (querySrc)
  * **Red** — destination node (queryDst)
  * **Orange** — nodes on the shortest path
  * **Blue** — all other nodes
* The bottom panel shows the path string and its total weight.

---

### Milestone 3 — Animated entity along the path (`sim.c`)

Adds a moving yellow entity that travels from source to destination along the Dijkstra path, on top of the static display from milestone 2.

* **Play / Stop / Restart button** in the bottom-right of the window starts, pauses, and restarts the animation. The spacebar also toggles play/pause.
* **State machine**: the entity is either `AT_NODE`, `ON_EDGE`, or `FINISHED`.
  * At the source node, it leaves immediately (no waiting).
  * At every **intermediate** node, it waits exactly **1 second** before moving on.
  * On an edge of weight `W`, the traversal is divided into **W jumps of 300 ms each**, so heavier edges take proportionally longer (`W × 300 ms` total).
  * On reaching the destination, an "Arrived at destination!" message is shown.
* The static graph (nodes, edges, weights, path highlight) remains visible behind the animation at all times.
* A **Status** indicator in the bottom panel shows whether the animation is Playing, Paused, or Arrived.

---

## Project structure

```
.
├── main.c        # milestone 1 — Dijkstra (terminal)
├── sim.c         # milestones 2 and 3 — GUI + animation
├── Makefile      # build targets for all milestones
├── input.txt     # sample graph
├── README.md     # this file
└── submission.txt
```
