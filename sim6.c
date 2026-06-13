/* ============================================================================
 *  OS Project  --  Milestone 6 : node mutual exclusion via POSIX semaphores
 * ----------------------------------------------------------------------------
 *  Builds on Milestone 5 (per-child pipes, children compute own Dijkstra).
 *
 *  New constraint: at most one traveler may be inside any given node at once.
 *
 *  Synchronization mechanism: unnamed POSIX semaphores (sem_init / sem_wait /
 *  sem_post) stored in an anonymous shared-memory mapping created before fork().
 *  One binary semaphore (initial value 1) per node.
 *
 *  Why unnamed semaphores in mmap(MAP_SHARED|MAP_ANONYMOUS), not sem_open:
 *    - Parent creates the array before fork(); children inherit the mapping
 *      because MAP_SHARED|MAP_ANONYMOUS pages are shared across fork(), unlike
 *      heap allocations which are copy-on-write.
 *    - No filesystem names to clean up (no /dev/shm entries).
 *    - No name collisions between concurrent test runs.
 *    - API is the same sem_wait/sem_post as named semaphores.
 *    - Destroyed with sem_destroy + munmap -- no sem_unlink.
 *
 *  Deadlock: impossible.  Each child holds at most one node semaphore at a
 *  time (acquire → 1 s stay → release → travel edge → next node).  The
 *  hold-and-wait condition for deadlock is never met.
 *
 *  Starvation: impossible.  POSIX sem_wait is fair; any blocked process will
 *  eventually be scheduled.  With one holder and a 1 s hold time, a waiter
 *  blocks at most (travelers - 1) seconds per node.
 *
 *  Early exit (window close): SIGTERM interrupts sem_wait (POSIX guarantee).
 *  The child dies without holding the semaphore; parent then waitpid()s it.
 *  No deadlock in cleanup.
 *
 *  Extended pipe message protocol (two types replace M5 MSG_ARRIVE):
 *    MSG_PATH    (0) -- unchanged from M5: child's computed path.
 *    MSG_WAITING (1) -- child finished edge traversal, calling sem_wait.
 *                       Parent: render traveler OUTSIDE node (hollow dot).
 *    MSG_ENTER   (2) -- child acquired semaphore, inside node.
 *                       Parent: render traveler INSIDE node; print terminal.
 *
 *  GUI visual distinction:
 *    INSIDE  (ANIM_AT_NODE,  pathIdx > 0): solid colored dot at node center.
 *    WAITING (ANIM_WAITING)              : white dot with colored border,
 *                                          offset outside the node circle.
 *    Occupied nodes get a red outline instead of white.
 *
 *  Build:
 *    GUI:      make milestone6            -> ./sim6   (needs raylib)
 *    Headless: make milestone6-headless   -> ./sim6_test (no raylib)
 *
 *  SIM_DEBUG=1 enables a parent-side trace on stderr.
 * ==========================================================================*/

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <semaphore.h>

#ifndef HEADLESS
#include <raylib.h>
#else
typedef struct { unsigned char r, g, b, a; } Color;
#endif

/* ---------------------------------------------------------------- limits -- */
#define WIN_W        1000
#define WIN_H         720
#define NODE_R         26
#define MAX_N          15
#define MAX_TRAVELERS  64
#define PIV   3.14159265f

/* travel timing */
#define NODE_WAIT  1.0   /* seconds inside each intermediate node  */
#define JUMP_TIME  0.3   /* seconds per unit edge weight           */

/* =================================  model  ================================ */

typedef struct { int dst; int weight; } Edge;
typedef struct { Edge *edges; int count; int capacity; } EdgeList;
typedef struct { int numNodes; int numEdges; EdgeList *adjList; } Graph;

typedef enum {
    ANIM_AT_NODE,   /* inside a node (holding lock) or at source before start */
    ANIM_WAITING,   /* waiting outside a node for the semaphore               */
    ANIM_ON_EDGE,   /* (unused for GUI positioning, kept for completeness)     */
    ANIM_FINISHED
} AnimState;

/* ---- pipe message types ---- */
#define MSG_PATH     0  /* child -> parent: computed path (sent once, first)   */
#define MSG_WAITING  1  /* child -> parent: waiting outside node (pre-sem)     */
#define MSG_ENTER    2  /* child -> parent: entered node (sem acquired)        */

typedef struct {
    int       type;
    /* MSG_PATH */
    int       plen;
    int       path[MAX_N];
    long long weight;
    /* MSG_WAITING / MSG_ENTER */
    int       current;    /* node index                                        */
    int       next;       /* next node (MSG_ENTER only); -1 = at destination  */
} TravelMsg;

typedef struct {
    int       src, dst;
    int       path[MAX_N];
    int       plen;
    long long weight;

    pid_t     pid;
    int       reaped;
    int       pipe_fd[2];   /* [0] = parent read, [1] = child write           */

    AnimState state;
    int       pathIdx;      /* index in path[] of the node we're at           */
    float     x, y;        /* on-screen position                              */
    Color     color;
} Traveler;

static float nx[MAX_N], ny[MAX_N];

/* Shared semaphore array: one binary semaphore per node, in mmap'd memory. */
static sem_t *node_sems = NULL;

static int g_debug = 0;
#define DBG(...) do { if (g_debug) { fprintf(stderr, __VA_ARGS__); fflush(stderr); } } while (0)

/* ============================  graph helpers  ============================= */

static void ginit(Graph *g) { g->numNodes = 0; g->numEdges = 0; g->adjList = NULL; }

static void gfree(Graph *g) {
    if (!g || !g->adjList) return;
    for (int i = 0; i < g->numNodes; i++) free(g->adjList[i].edges);
    free(g->adjList);
    g->adjList = NULL;
}

static int epush(EdgeList *el, int d, int w) {
    if (el->count == el->capacity) {
        int nc = (el->capacity == 0) ? 4 : el->capacity * 2;
        Edge *tmp = (Edge *)realloc(el->edges, (size_t)nc * sizeof(Edge));
        if (!tmp) return 0;
        el->edges = tmp; el->capacity = nc;
    }
    el->edges[el->count].dst = d;
    el->edges[el->count].weight = w;
    el->count++;
    return 1;
}

static int edge_weight(const Graph *g, int u, int v) {
    for (int k = 0; k < g->adjList[u].count; k++)
        if (g->adjList[u].edges[k].dst == v) return g->adjList[u].edges[k].weight;
    return 1;
}

/* O(N^2) Dijkstra -- fine for N <= MAX_N (15). */
static int dijkstra(const Graph *g, int src, int dst,
                    int *path, int *plen, long long *wt) {
    int N = g->numNodes;
    long long *dist = (long long *)malloc((size_t)N * sizeof(long long));
    int       *prev = (int *)malloc((size_t)N * sizeof(int));
    int       *vis  = (int *)calloc((size_t)N, sizeof(int));
    if (!dist || !prev || !vis) { free(dist); free(prev); free(vis); return 0; }

    for (int i = 0; i < N; i++) { dist[i] = LLONG_MAX; prev[i] = -1; }
    dist[src] = 0;

    for (int it = 0; it < N; it++) {
        int u = -1;
        for (int i = 0; i < N; i++)
            if (!vis[i] && dist[i] != LLONG_MAX && (u == -1 || dist[i] < dist[u])) u = i;
        if (u == -1) break;
        vis[u] = 1;
        for (int k = 0; k < g->adjList[u].count; k++) {
            int v = g->adjList[u].edges[k].dst, w = g->adjList[u].edges[k].weight;
            if (dist[u] + w < dist[v]) { dist[v] = dist[u] + w; prev[v] = u; }
        }
    }

    if (dist[dst] == LLONG_MAX) { free(dist); free(prev); free(vis); return 0; }

    int tmp[MAX_N], len = 0, cur = dst;
    while (cur != -1) { tmp[len++] = cur; if (cur == src) break; cur = prev[cur]; }
    for (int i = 0; i < len; i++) path[i] = tmp[len - 1 - i];
    *plen = len; *wt = dist[dst];
    free(dist); free(prev); free(vis);
    return 1;
}

/* ============================  input parsing  ============================= */

static int next_line(FILE *f, char *buf, size_t cap) {
    while (fgets(buf, (int)cap, f)) {
        char *h = strchr(buf, '#'); if (h) *h = '\0';
        char *p = buf;
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p != '\0') return 1;
    }
    return 0;
}

static int load_input(Graph *g, const char *fn, Traveler *trav, int *numTrav) {
    FILE *f = fopen(fn, "r");
    if (!f) { fprintf(stderr, "Error: cannot open file '%s'.\n", fn); return 0; }

    char line[256]; int N, M;

    if (!next_line(f, line, sizeof line)) {
        fprintf(stderr, "Error: empty file (expected 'N M').\n"); goto fail; }
    if (sscanf(line, "%d %d", &N, &M) != 2) {
        fprintf(stderr, "Error: bad header (expected 'N M').\n"); goto fail; }
    if (N <= 0 || N > MAX_N) {
        fprintf(stderr, "Error: node count must be 1..%d (got %d).\n", MAX_N, N); goto fail; }
    if (M < 0) {
        fprintf(stderr, "Error: edge count cannot be negative (got %d).\n", M); goto fail; }

    g->numNodes = N; g->numEdges = M;
    g->adjList = (EdgeList *)calloc((size_t)N, sizeof(EdgeList));
    if (!g->adjList) { fprintf(stderr, "Error: out of memory.\n"); goto fail; }

    for (int i = 0; i < M; i++) {
        int s, d, w;
        if (!next_line(f, line, sizeof line)) {
            fprintf(stderr, "Error: missing edge %d of %d.\n", i, M); goto fail; }
        if (sscanf(line, "%d %d %d", &s, &d, &w) != 3) {
            fprintf(stderr, "Error: bad edge %d (expected 'src dst weight').\n", i); goto fail; }
        if (s < 0 || d < 0 || w < 0) {
            fprintf(stderr, "Error: negative numbers not allowed (edge %d: %d %d %d).\n",
                    i, s, d, w); goto fail; }
        if (s >= N || d >= N) {
            fprintf(stderr, "Error: node index out of range at edge %d (src=%d dst=%d, N=%d).\n",
                    i, s, d, N); goto fail; }
        if (!epush(&g->adjList[s], d, w)) {
            fprintf(stderr, "Error: out of memory.\n"); goto fail; }
    }

    int T;
    if (!next_line(f, line, sizeof line)) {
        fprintf(stderr, "Error: missing travelers section (expected a count).\n"); goto fail; }
    if (sscanf(line, "%d", &T) != 1) {
        fprintf(stderr, "Error: bad travelers count.\n"); goto fail; }
    if (T <= 0) {
        fprintf(stderr, "Error: need at least one traveler (got %d).\n", T); goto fail; }
    if (T > MAX_TRAVELERS) {
        fprintf(stderr, "Error: too many travelers (%d > %d).\n", T, MAX_TRAVELERS); goto fail; }

    for (int i = 0; i < T; i++) {
        int s, d;
        if (!next_line(f, line, sizeof line)) {
            fprintf(stderr, "Error: missing traveler %d of %d.\n", i, T); goto fail; }
        if (sscanf(line, "%d %d", &s, &d) != 2) {
            fprintf(stderr, "Error: bad traveler %d (expected 'src dst').\n", i); goto fail; }
        if (s < 0 || d < 0) {
            fprintf(stderr, "Error: negative numbers not allowed (traveler %d: %d %d).\n",
                    i, s, d); goto fail; }
        if (s >= N || d >= N) {
            fprintf(stderr, "Error: traveler %d node index out of range (src=%d dst=%d, N=%d).\n",
                    i, s, d, N); goto fail; }
        memset(&trav[i], 0, sizeof(Traveler));
        trav[i].src = s; trav[i].dst = d;
        trav[i].pipe_fd[0] = trav[i].pipe_fd[1] = -1;
    }

    *numTrav = T;
    fclose(f);
    return 1;

fail:
    fclose(f);
    gfree(g);
    return 0;
}

/* ============================  layout / color  =========================== */

static void layout(int n) {
    float cx = WIN_W / 2.0f, cy = (WIN_H - 80) / 2.0f;
    if (n == 1) { nx[0] = cx; ny[0] = cy; return; }
    if (n <= 8) {
        float r = 220;
        for (int i = 0; i < n; i++) {
            float a = -PIV / 2 + 2 * PIV * i / n;
            nx[i] = cx + r * cosf(a); ny[i] = cy + r * sinf(a);
        }
    } else {
        int in = n / 2, ou = n - in; float r1 = 120, r2 = 255;
        for (int i = 0; i < in; i++) {
            float a = -PIV / 2 + 2 * PIV * i / in;
            nx[i] = cx + r1 * cosf(a); ny[i] = cy + r1 * sinf(a);
        }
        for (int i = 0; i < ou; i++) {
            float a = -PIV / 2 + 2 * PIV * i / ou;
            nx[in + i] = cx + r2 * cosf(a); ny[in + i] = cy + r2 * sinf(a);
        }
    }
}

static Color make_color(int i, int n) {
#ifdef HEADLESS
    (void)i; (void)n; Color c = { 0, 0, 0, 255 }; return c;
#else
    float hue = (n > 0) ? (360.0f * (float)i / (float)n) : 0.0f;
    return ColorFromHSV(hue, 0.72f, 1.00f);
#endif
}

/* ========================  process orchestration  ======================== */

static int all_finished(const Traveler *t, int n) {
    for (int i = 0; i < n; i++) if (t[i].state != ANIM_FINISHED) return 0;
    return 1;
}

/*
 * drain_pipes -- called each frame (non-blocking).
 * Processes messages from all child pipes:
 *   MSG_PATH    -> store path for GUI visualization.
 *   MSG_WAITING -> snap dot to node, set ANIM_WAITING (renders outside node).
 *   MSG_ENTER   -> set ANIM_AT_NODE (inside), print terminal line.
 */
static int drain_pipes(Traveler *trav, int n) {
    int activity = 0;
    for (int i = 0; i < n; i++) {
        if (trav[i].pipe_fd[0] < 0) continue;

        TravelMsg msg;
        ssize_t r;
        int done = 0;
        while (!done &&
               (r = read(trav[i].pipe_fd[0], &msg, sizeof msg)) == (ssize_t)sizeof msg) {
            activity = 1;
            switch (msg.type) {

            case MSG_PATH:
                trav[i].plen   = msg.plen;
                trav[i].weight = msg.weight;
                memcpy(trav[i].path, msg.path, (size_t)msg.plen * sizeof(int));
                DBG("PARENT: traveler %d MSG_PATH plen=%d weight=%lld\n",
                    i, msg.plen, msg.weight);
                if (msg.plen <= 1) {
                    trav[i].state = ANIM_FINISHED;
                    close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1;
                }
                done = 1;  /* only one MSG_PATH per pipe */
                break;

            case MSG_WAITING:
                /* Child arrived at node boundary, about to call sem_wait.
                 * Snap position to this node but mark as WAITING (outside). */
                trav[i].state = ANIM_WAITING;
                trav[i].x     = nx[msg.current];
                trav[i].y     = ny[msg.current];
                for (int k = 0; k < trav[i].plen; k++)
                    if (trav[i].path[k] == msg.current) { trav[i].pathIdx = k; break; }
                DBG("PARENT: traveler %d waiting outside node %d\n", i, msg.current);
                break;

            case MSG_ENTER:
                /* Child acquired the semaphore -- it is now inside the node. */
                trav[i].state = ANIM_AT_NODE;
                trav[i].x     = nx[msg.current];
                trav[i].y     = ny[msg.current];
                for (int k = 0; k < trav[i].plen; k++)
                    if (trav[i].path[k] == msg.current) { trav[i].pathIdx = k; break; }

                if (msg.next < 0) {
                    printf("[PID=%d] arrived at node %d | next node: DESTINATION\n",
                           (int)trav[i].pid, msg.current);
                    printf("[PID=%d] finished\n", (int)trav[i].pid);
                    fflush(stdout);
                    trav[i].state = ANIM_FINISHED;
                    close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1;
                    done = 1;
                } else {
                    printf("[PID=%d] arrived at node %d | next node: %d\n",
                           (int)trav[i].pid, msg.current, msg.next);
                    fflush(stdout);
                    DBG("PARENT: traveler %d entered node %d -> %d\n",
                        i, msg.current, msg.next);
                }
                break;
            }
        }

        /* EOF: child exited cleanly */
        if (r == 0 && trav[i].pipe_fd[0] >= 0) {
            trav[i].state = ANIM_FINISHED;
            close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1;
        }
        /* r == -1, errno == EAGAIN: no data yet -- normal non-blocking return */
    }
    return activity;
}

/* Kill any still-running children and reap all. */
static void cleanup_children(Traveler *trav, int n) {
    /* Close read ends first; children writing to a closed pipe get SIGPIPE. */
    for (int i = 0; i < n; i++) {
        if (trav[i].pipe_fd[0] >= 0) {
            close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1;
        }
    }
    /*
     * SIGTERM interrupts sem_wait (POSIX guarantee): a child blocked waiting
     * for a node dies without ever acquiring the semaphore -- no lock left
     * behind.  A child killed while holding the semaphore (during usleep)
     * does leave it locked, but cleanup_children kills ALL children before
     * sem_destroy is called, so no process is left waiting on a locked sem.
     * Safe for this design; not a general robustness claim.
     */
    for (int i = 0; i < n; i++) {
        if (!trav[i].reaped && trav[i].pid > 0) {
            if (trav[i].state != ANIM_FINISHED)
                kill(trav[i].pid, SIGTERM);
            waitpid(trav[i].pid, NULL, 0);
            trav[i].reaped = 1;
            DBG("PARENT: reaped child pid=%d (traveler %d)\n", (int)trav[i].pid, i);
        }
    }
    DBG("PARENT: all %d children reaped.\n", n);
}

/* ===============================  drawing  =============================== */
#ifndef HEADLESS

static void draw_arrow(int u, int v, Color c, float thick) {
    float dx = nx[v] - nx[u], dy = ny[v] - ny[u], l = sqrtf(dx * dx + dy * dy);
    if (l < 1) return;
    float ux = dx / l, uy = dy / l;
    float sx = nx[u] + ux * (NODE_R + 2), sy = ny[u] + uy * (NODE_R + 2);
    float ex = nx[v] - ux * (NODE_R + 2), ey = ny[v] - uy * (NODE_R + 2);
    DrawLineEx((Vector2){ sx, sy }, (Vector2){ ex, ey }, thick, c);
    float ar = 13, aw = 7, px = -uy, py = ux;
    DrawTriangle((Vector2){ ex, ey },
                 (Vector2){ ex - ux * ar + px * aw, ey - uy * ar + py * aw },
                 (Vector2){ ex - ux * ar - px * aw, ey - uy * ar - py * aw }, c);
}

static const char *state_word(const Traveler *T) {
    if (T->state == ANIM_FINISHED) return "Arrived";
    if (T->state == ANIM_WAITING)  return "Waiting";
    if (T->pathIdx > 0)            return "Inside";
    return "Start";
}

static void draw_frame(const Graph *g, Traveler *trav, int n, int paused) {
    Color CN  = { 55,  70, 110, 255 };
    Color CE  = { 60,  75, 115, 255 };
    Color CT  = { 210, 220, 240, 255 };
    Color CW  = { 130, 155, 200, 255 };
    Color OCC = { 255,  80,  80, 255 };   /* occupied node outline */

    BeginDrawing();
    ClearBackground((Color){ 12, 15, 26, 255 });

    for (int x = 0; x < WIN_W; x += 38)
        for (int y = 0; y < WIN_H - 80; y += 38)
            DrawCircle(x, y, 1, (Color){ 30, 38, 65, 255 });

    DrawText("OS Project - Milestone 6 (semaphore node locks)", 18, 12, 20,
             (Color){ 90, 115, 185, 255 });

    /* neutral edges + weights */
    for (int u = 0; u < g->numNodes; u++)
        for (int k = 0; k < g->adjList[u].count; k++) {
            int v = g->adjList[u].edges[k].dst, w = g->adjList[u].edges[k].weight;
            draw_arrow(u, v, CE, 1.6f);
            char ws[8]; snprintf(ws, sizeof ws, "%d", w);
            DrawText(ws, (int)((nx[u] + nx[v]) / 2 + 10),
                        (int)((ny[u] + ny[v]) / 2 - 10), 15, CW);
        }

    /* each traveler's route, tinted in its color */
    for (int i = 0; i < n; i++) {
        if (trav[i].plen < 2) continue;
        Color c = trav[i].color; c.a = 120;
        for (int e = 0; e < trav[i].plen - 1; e++)
            draw_arrow(trav[i].path[e], trav[i].path[e + 1], c, 3.0f);
    }

    /* nodes -- red outline when occupied, white when free */
    for (int i = 0; i < g->numNodes; i++) {
        /* check occupancy: a traveler is inside if ANIM_AT_NODE, pathIdx > 0,
         * and their path[pathIdx] == this node */
        int occ = 0;
        for (int t = 0; t < n; t++) {
            if (trav[t].state == ANIM_AT_NODE && trav[t].pathIdx > 0 &&
                trav[t].pathIdx < trav[t].plen &&
                trav[t].path[trav[t].pathIdx] == i) { occ = 1; break; }
        }
        DrawCircle((int)nx[i], (int)ny[i], NODE_R, CN);
        DrawCircleLines((int)nx[i], (int)ny[i], NODE_R, occ ? OCC : WHITE);
        /* extra glow ring when occupied */
        if (occ)
            DrawCircleLines((int)nx[i], (int)ny[i], NODE_R + 4,
                            (Color){ 255, 80, 80, 80 });
        char lb[8]; snprintf(lb, sizeof lb, "%d", i);
        DrawText(lb, (int)nx[i] - MeasureText(lb, 19) / 2, (int)ny[i] - 9, 19, CT);
    }

    /*
     * Traveler dots:
     *   ANIM_WAITING -> white dot with thick colored border, positioned OUTSIDE
     *                   the node circle (offset by traveler index so multiple
     *                   waiters at the same node don't overlap).
     *   All others   -> solid colored dot at (x, y).
     */
    for (int i = 0; i < n; i++) {
        if (trav[i].plen < 1) continue;

        if (trav[i].state == ANIM_WAITING) {
            float angle = (float)i / (float)(n > 1 ? n : 1) * 2.0f * PIV;
            float wx = trav[i].x + (NODE_R + 17.0f) * cosf(angle);
            float wy = trav[i].y + (NODE_R + 17.0f) * sinf(angle);
            DrawCircle((int)wx, (int)wy, 11, WHITE);
            DrawCircleLines((int)wx, (int)wy, 11, trav[i].color);
            DrawCircleLines((int)wx, (int)wy, 13, trav[i].color);
            char id[8]; snprintf(id, sizeof id, "%d", i);
            DrawText(id, (int)wx - MeasureText(id, 12) / 2, (int)wy - 6, 12,
                     trav[i].color);
        } else {
            DrawCircle((int)trav[i].x, (int)trav[i].y, 13, trav[i].color);
            DrawCircleLines((int)trav[i].x, (int)trav[i].y, 13, BLACK);
            char id[8]; snprintf(id, sizeof id, "%d", i);
            DrawText(id, (int)trav[i].x - MeasureText(id, 14) / 2,
                        (int)trav[i].y - 7, 14, BLACK);
        }
    }

    /* legend */
    int lx = 18, ly = 40;
    for (int i = 0; i < n && i < 12; i++) {
        DrawRectangle(lx, ly + i * 20 + 2, 14, 14, trav[i].color);
        DrawRectangleLines(lx, ly + i * 20 + 2, 14, 14, WHITE);
        char row[96];
        if (trav[i].plen == 0)
            snprintf(row, sizeof row, "T%d  %d->%d  (no path)", i, trav[i].src, trav[i].dst);
        else
            snprintf(row, sizeof row, "T%d  %d->%d  %s",
                     i, trav[i].src, trav[i].dst, state_word(&trav[i]));
        DrawText(row, lx + 20, ly + i * 20, 15, CT);
    }

    /* all-arrived banner */
    if (all_finished(trav, n)) {
        const char *msg = "All travelers arrived!";
        int mw = MeasureText(msg, 28);
        DrawRectangle(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46,
                      (Color){ 10, 50, 25, 230 });
        DrawRectangleLines(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46,
                           (Color){ 80, 220, 100, 255 });
        DrawText(msg, WIN_W / 2 - mw / 2, 33, 28, (Color){ 255, 240, 70, 255 });
    }

    /* bottom panel */
    DrawRectangle(0, WIN_H - 80, WIN_W, 80, (Color){ 20, 24, 40, 255 });
    DrawLine(0, WIN_H - 80, WIN_W, WIN_H - 80, (Color){ 45, 55, 90, 255 });
    int done = 0, waiting = 0;
    for (int i = 0; i < n; i++) {
        if (trav[i].state == ANIM_FINISHED) done++;
        if (trav[i].state == ANIM_WAITING)  waiting++;
    }
    char st[128];
    snprintf(st, sizeof st,
             "Travelers: %d   Inside: %d   Waiting: %d   Arrived: %d / %d   %s",
             n, n - done - waiting, waiting, done, n,
             paused ? "[PAUSED]" : "[RUNNING]");
    DrawText(st, 18, WIN_H - 64, 16, CT);
    DrawText("Node outline: RED = occupied   dot outside = waiting for entry",
             18, WIN_H - 38, 14, CW);
    EndDrawing();
}
#endif /* !HEADLESS */

/* ================================  main  ================================= */

int main(int argc, char *argv[]) {
    if (argc < 2) { fprintf(stderr, "Usage: %s <file>\n", argv[0]); return 1; }
    g_debug = (getenv("SIM_DEBUG") != NULL);

    Graph g; ginit(&g);
    Traveler trav[MAX_TRAVELERS];
    int n = 0;
    if (!load_input(&g, argv[1], trav, &n)) { gfree(&g); return 1; }
    layout(g.numNodes);

    /* ---- allocate one binary semaphore per node in shared memory ---- */
    node_sems = mmap(NULL, sizeof(sem_t) * MAX_N,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (node_sems == MAP_FAILED) {
        perror("mmap"); gfree(&g); return 1;
    }
    for (int i = 0; i < g.numNodes; i++) {
        if (sem_init(&node_sems[i], 1 /*pshared*/, 1 /*initial: unlocked*/) != 0) {
            perror("sem_init");
            for (int j = 0; j < i; j++) sem_destroy(&node_sems[j]);
            munmap(node_sems, sizeof(sem_t) * MAX_N);
            gfree(&g); return 1;
        }
    }
    DBG("PARENT: %d node semaphores initialised in shared memory.\n", g.numNodes);

    /* assign colors and set initial screen positions */
    for (int i = 0; i < n; i++) {
        trav[i].color   = make_color(i, n);
        trav[i].state   = ANIM_AT_NODE;
        trav[i].pathIdx = 0;
        trav[i].x       = nx[trav[i].src];
        trav[i].y       = ny[trav[i].src];
    }

    /* ---- create one pipe per traveler BEFORE forking ---- */
    for (int i = 0; i < n; i++) {
        if (pipe(trav[i].pipe_fd) != 0) {
            perror("pipe");
            for (int j = 0; j < i; j++) {
                close(trav[j].pipe_fd[0]); close(trav[j].pipe_fd[1]);
            }
            for (int j = 0; j < g.numNodes; j++) sem_destroy(&node_sems[j]);
            munmap(node_sems, sizeof(sem_t) * MAX_N);
            gfree(&g); return 1;
        }
    }

    fflush(NULL);

    /* ---- fork one child per traveler ---- */
    int forked = 0;
    for (int i = 0; i < n; i++) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("fork");
            for (int j = 0; j < forked; j++) {
                kill(trav[j].pid, SIGTERM); waitpid(trav[j].pid, NULL, 0);
            }
            for (int j = 0; j < n; j++) {
                close(trav[j].pipe_fd[0]); close(trav[j].pipe_fd[1]);
            }
            for (int j = 0; j < g.numNodes; j++) sem_destroy(&node_sems[j]);
            munmap(node_sems, sizeof(sem_t) * MAX_N);
            gfree(&g); return 1;
        }

        if (pid == 0) {
            /* ====== CHILD i ====== */
            /* Close all read ends and write ends of OTHER travelers. */
            for (int j = 0; j < n; j++) {
                close(trav[j].pipe_fd[0]);
                if (j != i) close(trav[j].pipe_fd[1]);
            }
            int wfd = trav[i].pipe_fd[1];

            /* Compute OWN Dijkstra path (graph inherited via fork). */
            int path[MAX_N]; int plen = 0; long long wt = 0;
            int src = trav[i].src, dst = trav[i].dst;

            if (src == dst) {
                path[0] = src; plen = 1; wt = 0;
            } else {
                if (!dijkstra(&g, src, dst, path, &plen, &wt))
                    plen = 0;
            }

            /* Send MSG_PATH for parent's route visualization. */
            TravelMsg pm; memset(&pm, 0, sizeof pm);
            pm.type = MSG_PATH; pm.plen = plen; pm.weight = wt;
            memcpy(pm.path, path, (size_t)plen * sizeof(int));
            { ssize_t wr = write(wfd, &pm, sizeof pm); (void)wr; }

            /*
             * Walk the path with node mutual exclusion:
             *   1. Travel the edge (usleep).
             *   2. MSG_WAITING  -- announce we're outside the node.
             *   3. sem_wait     -- acquire exclusive entry.
             *   4. MSG_ENTER    -- announce we're inside the node.
             *   5. usleep(NODE_WAIT) -- spend 1 s inside (intermediate only).
             *   6. sem_post     -- release the node.
             */
            for (int step = 1; step < plen; step++) {
                int prev_node = path[step - 1];
                int curr_node = path[step];
                int w = edge_weight(&g, prev_node, curr_node);
                if (w < 1) w = 1;

                /* edge traversal */
                usleep((useconds_t)(w * JUMP_TIME * 1000000.0));

                /* notify: waiting outside */
                TravelMsg wm; memset(&wm, 0, sizeof wm);
                wm.type = MSG_WAITING; wm.current = curr_node;
                { ssize_t wr = write(wfd, &wm, sizeof wm); (void)wr; }

                /* acquire node -- may block if another traveler is inside */
                sem_wait(&node_sems[curr_node]);

                /* notify: entered node */
                TravelMsg em; memset(&em, 0, sizeof em);
                em.type = MSG_ENTER; em.current = curr_node;
                em.next = (step < plen - 1) ? path[step + 1] : -1;
                { ssize_t wr = write(wfd, &em, sizeof em); (void)wr; }

                /* 1-second stay inside (not at destination) */
                if (step < plen - 1)
                    usleep((useconds_t)(NODE_WAIT * 1000000.0));

                /* release node */
                sem_post(&node_sems[curr_node]);
            }

            close(wfd);
            _exit(0);
        }

        /* ====== PARENT continues ====== */
        trav[i].pid = pid;
        forked++;
        DBG("PARENT: forked child pid=%d for traveler %d (%d -> %d)\n",
            (int)pid, i, trav[i].src, trav[i].dst);
    }

    /* Parent closes all write ends -- it only reads. */
    for (int i = 0; i < n; i++) {
        close(trav[i].pipe_fd[1]); trav[i].pipe_fd[1] = -1;
    }

    /*
     * Blocking read of one MSG_PATH per child.
     * Children send this immediately after fork (Dijkstra on N<=15 is
     * sub-millisecond), so these reads return quickly and prime the path
     * visualization before the GUI loop starts.
     */
    for (int i = 0; i < n; i++) {
        TravelMsg pm;
        ssize_t r = read(trav[i].pipe_fd[0], &pm, sizeof pm);
        if (r == (ssize_t)sizeof pm && pm.type == MSG_PATH) {
            trav[i].plen   = pm.plen;
            trav[i].weight = pm.weight;
            memcpy(trav[i].path, pm.path, (size_t)pm.plen * sizeof(int));
            if (pm.plen <= 1) {
                trav[i].state = ANIM_FINISHED;
                close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1;
            }
            DBG("PARENT: traveler %d MSG_PATH plen=%d\n", i, pm.plen);
        } else {
            trav[i].plen  = 0;
            trav[i].state = ANIM_FINISHED;
            if (trav[i].pipe_fd[0] >= 0) {
                close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1;
            }
        }
    }

    /* Switch read ends to non-blocking for the main loop. */
    for (int i = 0; i < n; i++) {
        if (trav[i].pipe_fd[0] < 0) continue;
        int fl = fcntl(trav[i].pipe_fd[0], F_GETFL, 0);
        fcntl(trav[i].pipe_fd[0], F_SETFL, fl | O_NONBLOCK);
    }

    DBG("PARENT: all MSG_PATH received; entering main loop.\n");

#ifdef HEADLESS
    /* ---- headless: select() loop until all travelers finish ---- */
    long guard = 0;
    while (!all_finished(trav, n)) {
        fd_set rset; FD_ZERO(&rset); int maxfd = 0;
        for (int i = 0; i < n; i++) {
            if (trav[i].pipe_fd[0] >= 0 && trav[i].state != ANIM_FINISHED) {
                FD_SET(trav[i].pipe_fd[0], &rset);
                if (trav[i].pipe_fd[0] > maxfd) maxfd = trav[i].pipe_fd[0];
            }
        }
        struct timeval tv = { 2, 0 };
        int sr = select(maxfd + 1, &rset, NULL, NULL, &tv);
        if (sr > 0) drain_pipes(trav, n);
        if (++guard > 1000000) { DBG("PARENT: guard limit hit\n"); break; }
    }

    cleanup_children(trav, n);

    /* Destroy semaphores and release shared memory. */
    for (int i = 0; i < g.numNodes; i++) sem_destroy(&node_sems[i]);
    munmap(node_sems, sizeof(sem_t) * MAX_N);

    gfree(&g);
    return 0;

#else
    /* ---- GUI: parent runs the raylib loop, drains pipes each frame ---- */
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(WIN_W, WIN_H, "OS Project - Milestone 6");
    SetTargetFPS(60);

    int paused = 0;
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (!paused) drain_pipes(trav, n);
        draw_frame(&g, trav, n, paused);
    }

    cleanup_children(trav, n);
    CloseWindow();

    /* Destroy semaphores and release shared memory. */
    for (int i = 0; i < g.numNodes; i++) sem_destroy(&node_sems[i]);
    munmap(node_sems, sizeof(sem_t) * MAX_N);

    gfree(&g);
    return 0;
#endif
}
