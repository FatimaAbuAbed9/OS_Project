/* ============================================================================
 *  OS Project  --  Milestone 7 : scheduler-driven node entry (FCFS / SJF)
 * ----------------------------------------------------------------------------
 *  Builds on Milestone 6 (per-node mutual exclusion). M6 used POSIX semaphores,
 *  which means the KERNEL decides which blocked traveler is woken up first --
 *  the application has no control over that order. M7 requires the opposite:
 *  the PARENT must own a waiting queue per node and explicitly choose the next
 *  traveler using a selectable scheduling algorithm (FCFS or SJF).
 *
 *  This requires replacing the semaphore with an explicit request/grant/release
 *  protocol over a SECOND pipe per traveler (parent -> child), since the
 *  parent must now be able to send a message ("you may enter"), not just
 *  receive one.
 *
 *  Pipe protocol (two directions):
 *    child -> parent (c2p):
 *      MSG_PATH    (0) -- computed path, sent once right after fork().
 *      MSG_REQUEST (1) -- "I want to enter node X next."  Carries:
 *                           next      = node after X (-1 = X is destination)
 *                           remaining = sum of edge weights from X to the
 *                                       traveler's destination (SJF key)
 *                           t_req     = CLOCK_MONOTONIC timestamp (FCFS key)
 *      MSG_RELEASE (2) -- "I am leaving node X."  Carries total_wait so far
 *                           (cumulative time spent queued, for the README's
 *                           wait-time comparison between schedulers).
 *    parent -> child (p2c):
 *      MSG_GRANT   (3) -- "You may enter the node you last requested."
 *
 *  Scheduler choice (-schd fcfs|sjf on the command line):
 *    FCFS -- the parent's per-node queue picks the request with the smallest
 *            t_req (earliest real request time). Fair: bounded wait of at
 *            most (waiters_ahead * hold_time).
 *    SJF  -- the parent's per-node queue picks the request with the smallest
 *            `remaining` (shortest remaining weighted trip to destination),
 *            breaking ties by t_req. Non-preemptive: the decision is made
 *            only at the moment a node frees up, exactly like classic SJF
 *            process scheduling.
 *
 *  Why "remaining trip distance" as the SJF key, not the next edge alone:
 *    It is the direct analogue of "remaining burst time" in CPU scheduling --
 *    each child already knows it for free from its own Dijkstra run, so no
 *    input-file changes are needed.  A single edge weight would only look at
 *    the next hop and ignore how much of the journey is actually left.
 *
 *  Deadlock: impossible, by the same structural argument as M6. A child holds
 *  at most one node "grant" at a time (request -> block for grant -> release
 *  -> next edge -> next request); hold-and-wait never occurs.
 *
 *  Starvation:
 *    FCFS -- impossible. Each node's queue is drained in real-time order;
 *            a waiter's position only improves over time, never worsens.
 *    SJF  -- NOT guaranteed, and that is the point of the comparison: a
 *            traveler with a long remaining trip can be repeatedly passed
 *            over by travelers with shorter remaining trips that keep
 *            arriving at the same node. This is the classic SJF tradeoff
 *            (low average wait, no fairness guarantee) and is demonstrated
 *            by tests/inputs/g_schedule.txt (see README).
 *
 *  Build:
 *    GUI:      make milestone7            -> ./sim7      (needs raylib)
 *    Headless: make milestone7-headless   -> ./sim7_test  (no raylib)
 *  Run:
 *    ./sim7 -schd fcfs <file_name>
 *    ./sim7 -schd sjf  <file_name>
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
#include <time.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/select.h>

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

typedef enum { SCHED_FCFS, SCHED_SJF } SchedAlgo;
static SchedAlgo g_algo = SCHED_FCFS;
static const char *algo_name(void) { return g_algo == SCHED_FCFS ? "FCFS" : "SJF"; }

typedef enum {
    ANIM_AT_NODE,   /* inside a node (granted) or at source before start */
    ANIM_WAITING,   /* queued, waiting for the scheduler to grant entry  */
    ANIM_ON_EDGE,   /* (unused for GUI positioning, kept for completeness)*/
    ANIM_FINISHED
} AnimState;

/* ---- pipe message types ---- */
#define MSG_PATH     0  /* child -> parent: computed path (sent once, first) */
#define MSG_REQUEST  1  /* child -> parent: requesting entry to a node       */
#define MSG_RELEASE  2  /* child -> parent: leaving a node                   */
#define MSG_GRANT    3  /* parent -> child: permission to enter granted node */

typedef struct {
    int       type;
    /* MSG_PATH */
    int       plen;
    int       path[MAX_N];
    long long weight;
    /* MSG_REQUEST / MSG_RELEASE / MSG_GRANT */
    int       current;      /* node being requested / granted / released    */
    int       next;         /* MSG_REQUEST only: node after current; -1=dest*/
    long long remaining;    /* MSG_REQUEST only: SJF key                    */
    double    t_req;        /* MSG_REQUEST only: FCFS key (CLOCK_MONOTONIC) */
    double    total_wait;   /* MSG_RELEASE only: cumulative wait so far     */
} TravelMsg;

typedef struct {
    int       src, dst;
    int       path[MAX_N];
    int       plen;
    long long weight;

    pid_t     pid;
    int       reaped;
    int       c2p_fd[2];   /* child -> parent: [0] parent reads, [1] child writes */
    int       p2c_fd[2];   /* parent -> child: [0] child reads,  [1] parent writes */

    AnimState state;
    int       pathIdx;      /* index in path[] of the node we're at           */
    float     x, y;         /* on-screen position                              */
    Color     color;

    int       pending_next;       /* set at grant time; -1 = granted node is destination */
    long long waiting_remaining;  /* set at request time, shown in GUI while waiting     */
    double    total_wait;         /* latest reported cumulative wait (stats)             */
} Traveler;

static float nx[MAX_N], ny[MAX_N];

/* Per-node FCFS/SJF waiting queue, owned and dispatched entirely by the parent. */
typedef struct {
    int       trav_idx;
    int       next;
    long long remaining;
    double    t_req;
} PendingReq;

typedef struct {
    PendingReq items[MAX_TRAVELERS];
    int        count;
    int        occupant;   /* traveler index currently inside; -1 = free */
} NodeQueue;

static NodeQueue node_q[MAX_N];

static int g_debug = 0;
#define DBG(...) do { if (g_debug) { fprintf(stderr, __VA_ARGS__); fflush(stderr); } } while (0)

static double now_mono(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

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

/* Sum of edge weights from path[step] to the end of the path (path[plen-1]). */
static long long remaining_weight(const Graph *g, const int *path, int step, int plen) {
    long long sum = 0;
    for (int k = step; k < plen - 1; k++) sum += edge_weight(g, path[k], path[k + 1]);
    return sum;
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
        trav[i].c2p_fd[0] = trav[i].c2p_fd[1] = -1;
        trav[i].p2c_fd[0] = trav[i].p2c_fd[1] = -1;
    }

    *numTrav = T;
    fclose(f);
    return 1;

fail:
    fclose(f);
    gfree(g);
    return 0;
}

/* =============================  CLI parsing  =============================== */

static int parse_args(int argc, char **argv, SchedAlgo *algo, const char **file) {
    *file = NULL;
    int got_schd = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-schd") == 0) {
            if (i + 1 >= argc) return 0;
            const char *name = argv[++i];
            if (strcmp(name, "fcfs") == 0) *algo = SCHED_FCFS;
            else if (strcmp(name, "sjf") == 0) *algo = SCHED_SJF;
            else {
                fprintf(stderr, "Error: unknown scheduler '%s' (use fcfs|sjf)\n", name);
                return 0;
            }
            got_schd = 1;
        } else {
            *file = argv[i];
        }
    }
    return got_schd && *file != NULL;
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
 * enqueue_request -- add a pending entry request to node `node`'s queue.
 * dispatch_node    -- if `node` is free and has at least one waiter, pick the
 *                      next one according to the active scheduler and grant it.
 *                      Printing the "[PID=...] arrived..." line happens here,
 *                      at the moment entry is granted -- the parent-side
 *                      equivalent of M6's MSG_ENTER handler.
 */
static void enqueue_request(int node, int trav_idx, int next, long long remaining, double t_req) {
    NodeQueue *q = &node_q[node];
    if (q->count >= MAX_TRAVELERS) return; /* defensive; cannot exceed total travelers */
    q->items[q->count].trav_idx  = trav_idx;
    q->items[q->count].next      = next;
    q->items[q->count].remaining = remaining;
    q->items[q->count].t_req     = t_req;
    q->count++;
	printf("[SCHEDULER] Traveler %d added to waiting queue for node %d (remaining=%lld, t_req=%.3f)\n", trav_idx, node, remaining, t_req);
}

static void dispatch_node(int node, Traveler *trav) {
    NodeQueue *q = &node_q[node];
    if (q->occupant >= 0 || q->count == 0) return;

    int pick = 0;
    for (int i = 1; i < q->count; i++) {
        if (g_algo == SCHED_FCFS) {
            if (q->items[i].t_req < q->items[pick].t_req) pick = i;
        } else { /* SJF: shortest remaining trip first; ties broken by t_req */
            if (q->items[i].remaining < q->items[pick].remaining ||
                (q->items[i].remaining == q->items[pick].remaining &&
                 q->items[i].t_req < q->items[pick].t_req))
                pick = i;
        }
    }

    PendingReq r = q->items[pick];
	printf("[SCHEDULER] %s selected Traveler %d for node %d (remaining=%lld)\n", algo_name(), r.trav_idx, node, r.remaining);
    for (int i = pick; i < q->count - 1; i++) q->items[i] = q->items[i + 1];
    q->count--;

    int ti = r.trav_idx;
    q->occupant = ti;
    trav[ti].state        = ANIM_AT_NODE;
    trav[ti].pending_next  = r.next;

    TravelMsg gm; memset(&gm, 0, sizeof gm);
    gm.type = MSG_GRANT; gm.current = node;
    { ssize_t wr = write(trav[ti].p2c_fd[1], &gm, sizeof gm); (void)wr; }

    if (r.next < 0) {
        printf("[PID=%d] arrived at node %d | next node: DESTINATION\n",
               (int)trav[ti].pid, node);
    } else {
        printf("[PID=%d] arrived at node %d | next node: %d\n",
               (int)trav[ti].pid, node, r.next);
    }
    fflush(stdout);
    DBG("PARENT[%s]: granted node %d to traveler %d (next=%d remaining=%lld t_req=%.3f)\n",
        algo_name(), node, ti, r.next, r.remaining, r.t_req);
}

/*
 * drain_pipes -- called each frame / select() wakeup (non-blocking c2p reads).
 *   MSG_PATH    -> store path for GUI visualization.
 *   MSG_REQUEST -> snap dot to node, set ANIM_WAITING, enqueue + try dispatch.
 *   MSG_RELEASE -> free the node, try to dispatch the next waiter; if this was
 *                  the traveler's final node, print "finished" and close fd.
 */
static int drain_pipes(Traveler *trav, int n) {
    int activity = 0;
    for (int i = 0; i < n; i++) {
        if (trav[i].c2p_fd[0] < 0) continue;

        TravelMsg msg;
        ssize_t r;
        int done = 0;
        while (!done &&
               (r = read(trav[i].c2p_fd[0], &msg, sizeof msg)) == (ssize_t)sizeof msg) {
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
                    close(trav[i].c2p_fd[0]); trav[i].c2p_fd[0] = -1;
                }
                done = 1;  /* only one MSG_PATH per pipe */
                break;

            case MSG_REQUEST:
                trav[i].state             = ANIM_WAITING;
                trav[i].waiting_remaining = msg.remaining;
                trav[i].x = nx[msg.current];
                trav[i].y = ny[msg.current];
                for (int k = 0; k < trav[i].plen; k++)
                    if (trav[i].path[k] == msg.current) { trav[i].pathIdx = k; break; }
                enqueue_request(msg.current, i, msg.next, msg.remaining, msg.t_req);
                dispatch_node(msg.current, trav);
                DBG("PARENT: traveler %d requested node %d (next=%d remaining=%lld)\n",
                    i, msg.current, msg.next, msg.remaining);
                break;

            case MSG_RELEASE: {
                int node = msg.current;
                trav[i].total_wait = msg.total_wait;
                node_q[node].occupant = -1;

                if (trav[i].pending_next < 0) {
                    printf("[PID=%d] finished\n", (int)trav[i].pid);
                    fflush(stdout);
                    DBG("PARENT[%s]: traveler %d total wait %.3fs\n",
                        algo_name(), i, trav[i].total_wait);
                    trav[i].state = ANIM_FINISHED;
                    close(trav[i].c2p_fd[0]); trav[i].c2p_fd[0] = -1;
                    done = 1;
                }
                dispatch_node(node, trav);
                break;
            }
            }
        }

        /* EOF: child exited cleanly */
        if (r == 0 && trav[i].c2p_fd[0] >= 0) {
            trav[i].state = ANIM_FINISHED;
            close(trav[i].c2p_fd[0]); trav[i].c2p_fd[0] = -1;
        }
        /* r == -1, errno == EAGAIN: no data yet -- normal non-blocking return */
    }
    return activity;
}

/* Kill any still-running children and reap all. */
static void cleanup_children(Traveler *trav, int n) {
    /* Close every pipe end the parent owns first. Closing p2c_fd[1] makes a
     * child blocked in read(rfd, ...) for a grant see EOF and exit cleanly;
     * closing c2p_fd[0] makes a writing child get SIGPIPE/EPIPE instead of
     * blocking forever. */
    for (int i = 0; i < n; i++) {
        if (trav[i].c2p_fd[0] >= 0) { close(trav[i].c2p_fd[0]); trav[i].c2p_fd[0] = -1; }
        if (trav[i].p2c_fd[1] >= 0) { close(trav[i].p2c_fd[1]); trav[i].p2c_fd[1] = -1; }
    }
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

static void print_wait_stats(const Traveler *trav, int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += trav[i].total_wait;
    printf("[STATS] scheduler=%s travelers=%d avg_wait=%.3fs\n",
           algo_name(), n, n > 0 ? sum / n : 0.0);
    fflush(stdout);
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

    char title[80];
    snprintf(title, sizeof title, "OS Project - Milestone 7 (scheduler: %s)", algo_name());
    DrawText(title, 18, 12, 20, (Color){ 90, 115, 185, 255 });

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
        int occ = (node_q[i].occupant >= 0);
        DrawCircle((int)nx[i], (int)ny[i], NODE_R, CN);
        DrawCircleLines((int)nx[i], (int)ny[i], NODE_R, occ ? OCC : WHITE);
        if (occ)
            DrawCircleLines((int)nx[i], (int)ny[i], NODE_R + 4,
                            (Color){ 255, 80, 80, 80 });
        char lb[8]; snprintf(lb, sizeof lb, "%d", i);
        DrawText(lb, (int)nx[i] - MeasureText(lb, 19) / 2, (int)ny[i] - 9, 19, CT);
    }

    /*
     * Traveler dots:
     *   ANIM_WAITING -> white dot with thick colored border, positioned OUTSIDE
     *                   the node circle (offset by traveler index), with a
     *                   small "remaining distance" label -- this is the SJF
     *                   key, shown so the scheduler's choice is visible.
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
            char rem[16]; snprintf(rem, sizeof rem, "r%lld", trav[i].waiting_remaining);
            DrawText(rem, (int)wx - MeasureText(rem, 11) / 2, (int)wy + 14, 11, CW);
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

    /* all-arrived banner, with average wait so the two schedulers can be compared */
    if (all_finished(trav, n)) {
        double sum = 0.0;
        for (int i = 0; i < n; i++) sum += trav[i].total_wait;
        char msg[80];
        snprintf(msg, sizeof msg, "All travelers arrived! (avg wait %.2fs, %s)",
                 n > 0 ? sum / n : 0.0, algo_name());
        int mw = MeasureText(msg, 24);
        DrawRectangle(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46,
                      (Color){ 10, 50, 25, 230 });
        DrawRectangleLines(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46,
                           (Color){ 80, 220, 100, 255 });
        DrawText(msg, WIN_W / 2 - mw / 2, 33, 24, (Color){ 255, 240, 70, 255 });
    }

    /* bottom panel */
    DrawRectangle(0, WIN_H - 80, WIN_W, 80, (Color){ 20, 24, 40, 255 });
    DrawLine(0, WIN_H - 80, WIN_W, WIN_H - 80, (Color){ 45, 55, 90, 255 });
    int done = 0, waiting = 0;
    for (int i = 0; i < n; i++) {
        if (trav[i].state == ANIM_FINISHED) done++;
        if (trav[i].state == ANIM_WAITING)  waiting++;
    }
    char st[160];
    snprintf(st, sizeof st,
             "Travelers: %d   Inside: %d   Waiting: %d   Arrived: %d / %d   Scheduler: %s   %s",
             n, n - done - waiting, waiting, done, n, algo_name(),
             paused ? "[PAUSED]" : "[RUNNING]");
    DrawText(st, 18, WIN_H - 64, 16, CT);
    DrawText("Node outline: RED = occupied   dot outside = queued   rN = remaining trip distance",
             18, WIN_H - 38, 14, CW);
    EndDrawing();
}
#endif /* !HEADLESS */

/* ================================  main  ================================= */

int main(int argc, char *argv[]) {
    const char *fname = NULL;
    if (!parse_args(argc, argv, &g_algo, &fname)) {
        fprintf(stderr, "Usage: %s -schd <fcfs|sjf> <file_name>\n", argv[0]);
        return 1;
    }
    g_debug = (getenv("SIM_DEBUG") != NULL);

    Graph g; ginit(&g);
    Traveler trav[MAX_TRAVELERS];
    int n = 0;
    if (!load_input(&g, fname, trav, &n)) { gfree(&g); return 1; }
    layout(g.numNodes);

    for (int i = 0; i < g.numNodes; i++) {
        node_q[i].count    = 0;
        node_q[i].occupant = -1;   /* must be explicit: 0 is a valid traveler index */
    }
    DBG("PARENT: scheduler=%s, %d node queues initialised.\n", algo_name(), g.numNodes);

    /* assign colors and set initial screen positions */
    for (int i = 0; i < n; i++) {
        trav[i].color   = make_color(i, n);
        trav[i].state   = ANIM_AT_NODE;
        trav[i].pathIdx = 0;
        trav[i].x       = nx[trav[i].src];
        trav[i].y       = ny[trav[i].src];
    }

    /* ---- create both pipes per traveler BEFORE forking ---- */
    for (int i = 0; i < n; i++) {
        if (pipe(trav[i].c2p_fd) != 0 || pipe(trav[i].p2c_fd) != 0) {
            perror("pipe");
            for (int j = 0; j <= i; j++) {
                if (trav[j].c2p_fd[0] >= 0) close(trav[j].c2p_fd[0]);
                if (trav[j].c2p_fd[1] >= 0) close(trav[j].c2p_fd[1]);
                if (trav[j].p2c_fd[0] >= 0) close(trav[j].p2c_fd[0]);
                if (trav[j].p2c_fd[1] >= 0) close(trav[j].p2c_fd[1]);
            }
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
                close(trav[j].c2p_fd[0]); close(trav[j].c2p_fd[1]);
                close(trav[j].p2c_fd[0]); close(trav[j].p2c_fd[1]);
            }
            gfree(&g); return 1;
        }

        if (pid == 0) {
            /* ====== CHILD i ====== */
            /* Each side closes the fds it does not need; only traveler i's own
             * write end of c2p and read end of p2c stay open in this child. */
            for (int j = 0; j < n; j++) {
                close(trav[j].c2p_fd[0]);
                close(trav[j].p2c_fd[1]);
                if (j != i) { close(trav[j].c2p_fd[1]); close(trav[j].p2c_fd[0]); }
            }
            int wfd = trav[i].c2p_fd[1];
            int rfd = trav[i].p2c_fd[0];

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

            double total_wait = 0.0;

            /*
             * Walk the path under scheduler control:
             *   1. Travel the edge (usleep).
             *   2. MSG_REQUEST  -- ask the parent for entry; carries next node,
             *                      remaining trip distance, and a timestamp.
             *   3. block on read(rfd, ...) for MSG_GRANT.
             *   4. usleep(NODE_WAIT) -- spend 1 s inside (intermediate only).
             *   5. MSG_RELEASE  -- tell the parent we are leaving.
             */
            for (int step = 1; step < plen; step++) {
                int prev_node = path[step - 1];
                int curr_node = path[step];
                int w = edge_weight(&g, prev_node, curr_node);
                if (w < 1) w = 1;

                /* edge traversal */
                usleep((useconds_t)(w * JUMP_TIME * 1000000.0));

                int next = (step < plen - 1) ? path[step + 1] : -1;
                long long remaining = remaining_weight(&g, path, step, plen);
                double t_req = now_mono();

                TravelMsg rq; memset(&rq, 0, sizeof rq);
                rq.type = MSG_REQUEST; rq.current = curr_node;
                rq.next = next; rq.remaining = remaining; rq.t_req = t_req;
                { ssize_t wr = write(wfd, &rq, sizeof rq); (void)wr; }

                /* block until the scheduler grants this node */
                TravelMsg gr;
                ssize_t rr = read(rfd, &gr, sizeof gr);
                if (rr != (ssize_t)sizeof gr) break;  /* parent gone (shutdown) */

                total_wait += now_mono() - t_req;

                /* 1-second stay inside (not at destination) */
                if (step < plen - 1)
                    usleep((useconds_t)(NODE_WAIT * 1000000.0));

                TravelMsg rel; memset(&rel, 0, sizeof rel);
                rel.type = MSG_RELEASE; rel.current = curr_node; rel.total_wait = total_wait;
                { ssize_t wr = write(wfd, &rel, sizeof rel); (void)wr; }
            }

            close(wfd);
            close(rfd);
            _exit(0);
        }

        /* ====== PARENT continues ====== */
        trav[i].pid = pid;
        forked++;
        DBG("PARENT: forked child pid=%d for traveler %d (%d -> %d)\n",
            (int)pid, i, trav[i].src, trav[i].dst);
    }

    /* Parent only reads c2p and only writes p2c. */
    for (int i = 0; i < n; i++) {
        close(trav[i].c2p_fd[1]); trav[i].c2p_fd[1] = -1;
        close(trav[i].p2c_fd[0]); trav[i].p2c_fd[0] = -1;
    }

    /*
     * Blocking read of one MSG_PATH per child.
     * Children send this immediately after fork (Dijkstra on N<=15 is
     * sub-millisecond), so these reads return quickly and prime the path
     * visualization before the GUI loop starts.
     */
    for (int i = 0; i < n; i++) {
        TravelMsg pm;
        ssize_t r = read(trav[i].c2p_fd[0], &pm, sizeof pm);
        if (r == (ssize_t)sizeof pm && pm.type == MSG_PATH) {
            trav[i].plen   = pm.plen;
            trav[i].weight = pm.weight;
            memcpy(trav[i].path, pm.path, (size_t)pm.plen * sizeof(int));
            if (pm.plen <= 1) {
                trav[i].state = ANIM_FINISHED;
                close(trav[i].c2p_fd[0]); trav[i].c2p_fd[0] = -1;
            }
            DBG("PARENT: traveler %d MSG_PATH plen=%d\n", i, pm.plen);
        } else {
            trav[i].plen  = 0;
            trav[i].state = ANIM_FINISHED;
            if (trav[i].c2p_fd[0] >= 0) {
                close(trav[i].c2p_fd[0]); trav[i].c2p_fd[0] = -1;
            }
        }
    }

    /* Switch c2p read ends to non-blocking for the main loop. */
    for (int i = 0; i < n; i++) {
        if (trav[i].c2p_fd[0] < 0) continue;
        int fl = fcntl(trav[i].c2p_fd[0], F_GETFL, 0);
        fcntl(trav[i].c2p_fd[0], F_SETFL, fl | O_NONBLOCK);
    }

    DBG("PARENT: all MSG_PATH received; entering main loop.\n");

#ifdef HEADLESS
    /* ---- headless: select() loop until all travelers finish ---- */
    long guard = 0;
    while (!all_finished(trav, n)) {
        fd_set rset; FD_ZERO(&rset); int maxfd = 0;
        for (int i = 0; i < n; i++) {
            if (trav[i].c2p_fd[0] >= 0 && trav[i].state != ANIM_FINISHED) {
                FD_SET(trav[i].c2p_fd[0], &rset);
                if (trav[i].c2p_fd[0] > maxfd) maxfd = trav[i].c2p_fd[0];
            }
        }
        struct timeval tv = { 2, 0 };
        int sr = select(maxfd + 1, &rset, NULL, NULL, &tv);
        if (sr > 0) drain_pipes(trav, n);
        if (++guard > 1000000) { DBG("PARENT: guard limit hit\n"); break; }
    }

    print_wait_stats(trav, n);
    cleanup_children(trav, n);
    gfree(&g);
    return 0;

#else
    /* ---- GUI: parent runs the raylib loop, drains pipes each frame ---- */
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(WIN_W, WIN_H, "OS Project - Milestone 7");
    SetTargetFPS(60);

    int paused = 0;
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (!paused) drain_pipes(trav, n);
        draw_frame(&g, trav, n, paused);
    }

    print_wait_stats(trav, n);
    cleanup_children(trav, n);
    CloseWindow();

    gfree(&g);
    return 0;
#endif
}
