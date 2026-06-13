# OS Project — Graph Movement Simulation

Operating Systems course project: a simulation of an entity moving on a directed weighted graph along the shortest path computed by Dijkstra's algorithm.

The graph is loaded from a text file, the shortest path between a source and destination is computed, and an animated entity traverses the path in real time using the **raylib** graphics library.

---

## Build

The project uses a single `Makefile` with one target per milestone.

```bash
make milestone1           # builds the ./dijkstra binary (milestone 1)
make milestone2           # builds the ./sim binary (milestones 2 and 3)
make milestone3           # same as milestone2 — same binary, more features
make milestone4           # builds the ./sim binary for milestone 4 (multiple travelers)
make milestone5           # builds the ./sim5 binary for milestone 5 (pipe IPC)
make milestone5-headless  # builds the ./sim5_test headless binary for milestone 5
make milestone6           # builds the ./sim6 binary for milestone 6 (semaphore node locks)
make milestone6-headless  # builds the ./sim6_test headless binary for milestone 6
make test                 # builds the headless test binary and runs the milestone-4 tests
make clean                # remove all compiled binaries
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

### Milestone 5 — Children compute own routes, report via pipe (`sim5.c`)

Restructures milestone 4 so that **each child independently computes its own
Dijkstra path** and **reports its position to the parent** each time it arrives at a
new node.

#### IPC mechanism: one anonymous pipe per child

A dedicated `pipe()` is created for each traveler before `fork()`.  The child holds
the write end; the parent holds the read end.  All other ends are closed in each
process after forking.

**Why pipes over shared memory:**

Pipes provide **built-in synchronization**: a `read()` on an empty pipe blocks until
data arrives, and the kernel guarantees FIFO ordering with no additional locks.
Shared memory gives no such guarantees — the reader and writer must coordinate with
a mutex or semaphore, and the programmer must manage ordering manually.  For this
problem (discrete, ordered, child-to-parent events) that extra machinery adds
complexity with no benefit.

| Criterion | Pipes | Shared memory |
|-----------|-------|---------------|
| Synchronization | Built-in: `read()` blocks until data is ready; no locks needed | Manual: requires mutex/semaphore to prevent races |
| Ordering | FIFO by kernel guarantee | Not guaranteed; reader must poll or use a flag |
| API familiarity | Already used in M4 (startup barrier); `pipe()`, `read()`, `write()` | New API: `shmget`/`shmat` or `mmap(MAP_SHARED)` |
| Traffic direction | One writer (child), one reader (parent) per pipe — no contention | Shared by all; every access needs protection |
| Message model | Discrete fixed-size events — the natural pipe use-case | Better for bulk data or random-access by multiple readers |
| Isolation | One pipe per child; travelers cannot interfere with each other | Shared region requires careful layout and index management |

**Message protocol** (`TravelMsg`, fixed size, < PIPE_BUF → atomic writes):

- `MSG_PATH` — sent once immediately after the child computes Dijkstra; carries the
  full path array and length so the parent can draw the colored route visualization.
- `MSG_ARRIVE` — sent once per node arrival (after sleeping `edge_weight × 300 ms`
  for edge traversal, then `1 s` at each intermediate node); carries `current` (node
  just reached) and `next` (next node, or `-1` at the destination).

#### Child lifecycle

1. Close all pipe read ends and other travelers' write ends.
2. Compute Dijkstra on the graph inherited from the parent via `fork()`.
3. Send one `MSG_PATH`.
4. For each hop: `usleep(edge_weight × 300 ms)`, send `MSG_ARRIVE`, then
   `usleep(1 s)` at intermediate nodes.
5. Close write end and `_exit(0)`.  No `SIGTERM` from parent needed.

#### Parent lifecycle

1. Creates pipes, forks children.
2. Closes all write ends.
3. Blocking read of one `MSG_PATH` per child to prime the GUI path visualization.
4. Switches read ends to `O_NONBLOCK`.
5. GUI loop calls `drain_pipes()` each frame; headless mode uses `select()`.
6. On `MSG_ARRIVE`: snaps traveler dot to the reported node and prints:
   ```
   [PID] arrived at node X | next node: Y
   [PID] arrived at node X | next node: (destination)
   ```
7. On window close or all arrived: `SIGTERM` any stragglers, `waitpid()` all.

#### Example run

```
$ make milestone5
$ ./sim5 tests/inputs/g_page3.txt
[1101] arrived at node 2 | next node: 3
[1102] arrived at node 1 | next node: 3
[1101] arrived at node 3 | next node: 4
[1102] arrived at node 3 | next node: (destination)
[1101] arrived at node 4 | next node: (destination)
```

A GUI window opens showing all travelers as colored dots that snap between nodes
as messages are received.  Space bar pauses/resumes.

---

### Milestone 6 — Node mutual exclusion via POSIX semaphores (`sim6.c`)

Adds a synchronization constraint on top of Milestone 5: **at most one traveler
may be inside any given node at the same time.**  Each node is a critical section
protected by a binary semaphore.

#### Synchronization mechanism: unnamed POSIX semaphores in shared memory

One `sem_t` per node is stored in an anonymous shared-memory region created with
`mmap(MAP_SHARED | MAP_ANONYMOUS)` before `fork()`.  Each semaphore is initialized
with `sem_init(&sem, 1 /*pshared*/, 1 /*unlocked*/)`.

**Why unnamed semaphores in `mmap`, not `sem_open` (named semaphores):**

| Criterion | Unnamed in `mmap` | Named (`sem_open`) |
|-----------|-------------------|--------------------|
| Setup | `mmap` before `fork`; children inherit the mapping | `sem_open` by name in every process |
| Cleanup | `sem_destroy` + `munmap` — no filesystem entries | `sem_unlink` required; entries linger if process crashes |
| Name conflicts | None — anonymous mapping | Concurrent test runs can collide on the same name |
| Scope | Private to this process tree | Visible to any process on the system |
| API | Identical `sem_wait` / `sem_post` once mapped | Identical `sem_wait` / `sem_post` after open |

The `MAP_SHARED | MAP_ANONYMOUS` mapping is shared across `fork()` — unlike heap
allocations which are copy-on-write.  So every child and the parent operate on the
same physical pages, making the semaphores truly shared without any additional
setup in the child.

#### No deadlock

Each child holds **at most one** node semaphore at a time:

```
acquire sem(N)  →  sleep 1 s  →  release sem(N)  →  travel edge  →  acquire sem(N+1) ...
```

Because no child ever holds two semaphores simultaneously, the *hold-and-wait*
condition for deadlock is never satisfied.

#### No starvation

POSIX `sem_wait` is fair: every blocked process will eventually be scheduled.
With one holder at a time and a fixed 1 s hold duration, a waiter blocks at most
*(travelers − 1) × 1 s* before entering.

#### Extended pipe message protocol

Two new message types replace M5's `MSG_ARRIVE`:

| Type | When sent | Parent action |
|------|-----------|---------------|
| `MSG_WAITING` (1) | After edge traversal, before `sem_wait` | Set `ANIM_WAITING`; render dot **outside** node |
| `MSG_ENTER`   (2) | After `sem_wait` returns (lock acquired)  | Set `ANIM_AT_NODE`; print terminal line |

`MSG_PATH` (0) is unchanged.

#### Child walk sequence per hop

```
usleep(edge_weight × 300 ms)     -- edge traversal
write MSG_WAITING(curr_node)     -- announce: queued outside
sem_wait(&node_sems[curr_node])  -- acquire critical section (may block)
write MSG_ENTER(curr_node, next) -- announce: inside node
usleep(1 s)                      -- stay (intermediate nodes only)
sem_post(&node_sems[curr_node])  -- release critical section
```

#### Early-exit safety (window closed mid-run)

POSIX guarantees that `sem_wait` is interrupted by signals.  When the parent sends
`SIGTERM` to a child blocked in `sem_wait`, the child terminates immediately
**without** acquiring the semaphore — no lock is left behind.

A child `SIGTERM`'d while **holding** the semaphore (during its `usleep` inside the
critical section) does leave the semaphore locked.  This is only safe here because
`cleanup_children` kills **all** children before `sem_destroy` is called, so no
process is ever left waiting on a locked semaphore.  This is a cleanup-path
property of this specific design, not a general semaphore robustness guarantee: a
long-lived server that needed to continue after a partial child failure would require
a different strategy (e.g., a watchdog that posts the semaphore on child death).

#### GUI visual distinctions

- **Inside** (`ANIM_AT_NODE`, `pathIdx > 0`): solid colored dot at node center.
- **Waiting** (`ANIM_WAITING`): white dot with thick colored border, positioned
  outside the node circle at a per-traveler angle (so multiple waiters at the same
  node spread around it without overlapping).
- **Occupied node**: red outline ring instead of white, plus a faint red glow.
  Free nodes: white outline.
- Bottom panel shows live count: *Inside: N   Waiting: N   Arrived: N*.

#### Example run

```
$ make milestone6
$ ./sim6 tests/inputs/g_page3.txt
[PID=1301] arrived at node 2 | next node: 3
[PID=1302] arrived at node 1 | next node: 3
[PID=1302] arrived at node 3 | next node: DESTINATION    <- 1302 entered first
[PID=1302] finished
[PID=1301] arrived at node 3 | next node: DESTINATION    <- 1301 waited, then entered
[PID=1301] finished
```

*(Order of arrivals at the same node will vary by run; mutual exclusion is always
enforced.)*

```
$ make milestone6-headless
$ ./sim6_test tests/inputs/g_page3.txt
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
├── sim5.c        # milestone 5 — children compute own routes, pipe IPC
├── sim6.c        # milestone 6 — node mutual exclusion via POSIX semaphores
├── Makefile      # build targets for all milestones
├── input.txt     # sample graph
├── README.md     # this file
└── tests/
    ├── run_tests.sh           # milestone-4 automated test suite
    ├── MANUAL_GUI_CHECKLIST.md# visual checks for the GUI-only requirements
    └── inputs/                # test fixtures (valid + invalid)
```
