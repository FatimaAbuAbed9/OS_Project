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
make milestone4   # builds the ./sim binary for milestone 4 (multiple travelers)
make test         # builds the headless test binary and runs the milestone-4 tests
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

### Extended format (milestone 4)

Milestone 4 replaces the single `src dst` query line with a **travelers section**: a
count `T`, then `T` lines of `source destination`. Lines beginning with `#` and blank
lines are ignored, so the `# graph definition` / `# travelers` headers are optional.

```
N M             <- number of nodes, number of edges
src dst weight  <- M edge lines
...
T               <- number of travelers
src dst         <- T lines, one per traveler
...
```

Example (`tests/inputs/g_page3.txt`):
```
# graph definition
5 7
0 1 4
0 2 2
1 3 5
2 1 1
2 3 8
3 4 2
1 4 6
# travelers
2
0 4
2 3
```

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

### Milestone 4 — Multiple processes and a parent process (`sim4.c`)

Moves from a single traveler to **several travelers moving at the same time**. The
**parent** process does all the work — it reads the (extended) input file, computes a
Dijkstra path for every traveler, drives the raylib loop, and draws every traveler.
The **children** only need to "live" for as long as they are travelling.

* **Extended input file** adds a travelers section after the graph (see format below):
  a count `T`, then `T` lines of `source destination`.
* **Parent process:**
  * reads the file and computes the Dijkstra path for each traveler **before** forking;
  * creates one child per traveler with **`fork()`**;
  * runs the raylib loop and draws all travelers, **each in a distinct color** (hues
    spread evenly around the color wheel, plus a legend);
  * when a traveler reaches its destination, **sends `SIGTERM`** to that traveler's
    child and **reaps it** (`waitpid`);
  * **waits for every child** before exiting (a final cleanup pass kills and reaps any
    child still alive if the window is closed early), so no zombies are left.
* **Child processes:** each child prints `[<pid>] started` once, then sleeps
  (`pause()`) doing nothing until the parent terminates it.
* All travellers' trips happen **in parallel** on screen, not one after another.

**IPC tool used:** a tiny pipe acts purely as a **startup barrier** — each child writes
one byte right after printing `started`, and the parent waits to read one byte per child
before starting the simulation. This guarantees every `started` line is printed before
the parent can signal anyone. (It carries no path data; milestone 5 is where children
compute and report their own paths.)

**Signal used:** `SIGTERM` (default disposition terminates the child); the parent reaps
each child with `waitpid`.

Example run:
```
$ make milestone4
$ ./sim tests/inputs/g_page3.txcd ~/Downloads/OS_Project-maint
[1021] started
[1022] started        # a GUI window opens with two differently-colored travelers
```

---

## Testing

Milestone 4 ships with an automated test suite plus a manual checklist for the visual
parts.

```bash
make test                       # builds the headless binary and runs all checks
# or directly:
bash tests/run_tests.sh
```

Because a GUI cannot be tested automatically, `sim4.c` has a **headless build**
(`make milestone4-headless` → `./sim_test`) that runs the *exact same* fork / signal /
wait orchestration on a virtual clock, with no raylib and no display. Setting the
environment variable `SIM_DEBUG=1` makes the parent print a trace of its actions
(fork / paths / signals / reaps) to stderr; this is OFF by default, so the normal graded
run only prints the children's `started` lines.

The suite (`tests/run_tests.sh`, 64 checks) covers:
* **Parsing** — valid files with and without `#` comment lines, and every error path
  (missing file, bad header, negative numbers, out-of-range nodes/travelers, zero
  travelers, too many nodes/travelers). Invalid input must be rejected **before** any
  child is forked.
* **Dijkstra** — exact shortest paths for each traveler, including `src == dst` and
  unreachable destinations.
* **Processes** — one child forked per traveler, each prints `started` with a distinct
  PID, the parent signals and reaps every child (no zombies), and the parent waits for
  all children without hanging.
* **Parallelism** — at least two travelers start moving at virtual time `0.00`, proving
  trips overlap rather than running sequentially.

The visual-only requirements (distinct colors on screen, simultaneous movement) are in
[`tests/MANUAL_GUI_CHECKLIST.md`](tests/MANUAL_GUI_CHECKLIST.md).

---

## Project structure

```
.
├── main.c        # milestone 1 — Dijkstra (terminal)
├── sim.c         # milestones 2 and 3 — GUI + animation
├── sim4.c        # milestone 4 — multiple travelers (fork + signals + GUI)
├── Makefile      # build targets for all milestones
├── input.txt     # sample graph
├── README.md     # this file
└── tests/
    ├── run_tests.sh           # milestone-4 automated test suite
    ├── MANUAL_GUI_CHECKLIST.md# visual checks for the GUI-only requirements
    └── inputs/                # test fixtures (valid + invalid)
```
