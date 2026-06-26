/* ============================================================================
 *  OS Project  --  Milestone 5 : children compute own routes, report via pipe
 * ----------------------------------------------------------------------------
 *  Architecture change from M4:
 *    - Each CHILD independently computes its own Dijkstra route (inheriting
 *      the graph from the parent's address space after fork()).
 *    - Children walk their route, sleeping edge_weight * 300 ms per hop, and
 *      write one TravelMsg to their dedicated pipe each time they arrive at a
 *      new node.  They exit cleanly when done.
 *    - The PARENT reads these messages (non-blocking in the GUI loop, select()
 *      in headless mode), snaps each traveler's dot to the reported node, and
 *      prints "[PID] arrived at node X | next node: Y" to the terminal.
 *
 *  IPC mechanism chosen: one anonymous pipe per child (child writes, parent
 *  reads).  Rationale:
 *    - Already the established pattern in this codebase (M4 startup barrier).
 *    - Node-arrival events are discrete, ordered, and strictly child-to-parent,
 *      which is exactly the use case pipes were designed for.
 *    - No synchronization primitives (mutexes / semaphores) required, unlike
 *      shared memory.
 *    - One pipe per child avoids multi-writer races and keeps each traveler's
 *      message stream independent.
 *    - All messages are a fixed-size TravelMsg struct (< PIPE_BUF), so writes
 *      are atomic and reads always align to message boundaries.
 *
 *  Message protocol (two types, same fixed-size struct, tagged by `type`):
 *    MSG_PATH  -- sent once, immediately after child computes Dijkstra; gives
 *                 the parent the path for GUI route visualization.
 *    MSG_ARRIVE -- sent once per node arrival; triggers terminal print and
 *                  GUI position snap.
 *
 *  Build (see Makefile):
 *    GUI:      make milestone5            -> ./sim5   (needs raylib)
 *    Headless: make milestone5-headless   -> ./sim5_test (no raylib)
 *
 *  Set environment variable SIM_DEBUG=1 for a parent-side trace on stderr.
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

/* animation / travel timing (children sleep these durations between writes) */
#define NODE_WAIT  1.0         /* seconds to wait at each intermediate node   */
#define JUMP_TIME  0.3         /* seconds per unit edge weight                */

/* =================================  model  ================================ */

typedef struct { int dst; int weight; } Edge;
typedef struct { Edge *edges; int count; int capacity; } EdgeList;
typedef struct { int numNodes; int numEdges; EdgeList *adjList; } Graph;

typedef enum { ANIM_AT_NODE, ANIM_ON_EDGE, ANIM_FINISHED } AnimState;

/* ---- pipe message ---- */
#define MSG_PATH    0   /* child -> parent: computed path (sent once, first)  */
#define MSG_ARRIVE  1   /* child -> parent: arrived at a new node             */
#define MSG_STUCK   2   /* child -> parent: no path found */
typedef struct {
    int       type;            /* MSG_PATH or MSG_ARRIVE                      */
    /* MSG_PATH fields */
    int       plen;
    int       path[MAX_N];
    long long weight;
    /* MSG_ARRIVE fields */
    int       current;         /* node just arrived at                        */
    int       next;            /* next node in path; -1 = at destination      */
    int stuck;              /* 1 if no path found */

} TravelMsg;

typedef struct {
    int       src, dst;
    int       path[MAX_N];     /* filled when MSG_PATH is received            */
    int       plen;
    long long weight;

    pid_t     pid;
    int       reaped;
    int       pipe_fd[2];      /* [0] = parent read end, [1] = child write end */

    AnimState state;
    int       pathIdx;         /* index in path[] of current node             */
    float     x, y;           /* on-screen position                          */
    Color     color;
} Traveler;

static float nx[MAX_N], ny[MAX_N];

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
 * Reads every available TravelMsg from each child's pipe:
 *   MSG_PATH  -> store path in Traveler for GUI route visualization.
 *   MSG_ARRIVE -> snap dot to reported node, print terminal line.
 * Returns 1 if any message was processed (useful for headless busy-wait guard).
 */
static int drain_pipes(Traveler *trav, int n) {
    int activity = 0;
    for (int i = 0; i < n; i++) {
        if (trav[i].pipe_fd[0] < 0) continue;

        TravelMsg msg;
        ssize_t r;
        while ((r = read(trav[i].pipe_fd[0], &msg, sizeof msg)) == (ssize_t)sizeof msg) {
            activity = 1;

            if (msg.type == MSG_PATH) {
                trav[i].plen   = msg.plen;
                trav[i].weight = msg.weight;
                memcpy(trav[i].path, msg.path, (size_t)msg.plen * sizeof(int));
                DBG("PARENT: traveler %d path received (plen=%d weight=%lld)\n",
                    i, msg.plen, msg.weight);
                /* src==dst or unreachable: mark finished immediately */
                if (msg.plen <= 1) {
                    trav[i].state = ANIM_FINISHED;
                    close(trav[i].pipe_fd[0]);
                    trav[i].pipe_fd[0] = -1;
                }
                break; /* only one MSG_PATH per pipe; switch to ARRIVE reads */
            }
            if (msg.type == MSG_STUCK) {
    printf("[PID=%d] STUCK: no path found!\n", (int)trav[i].pid);
    fflush(stdout);
    trav[i].state = ANIM_FINISHED;
    close(trav[i].pipe_fd[0]);
    trav[i].pipe_fd[0] = -1;
    break;
}

            /* MSG_ARRIVE */
            trav[i].x = nx[msg.current];
            trav[i].y = ny[msg.current];
            /* update pathIdx to the arrived node */
            for (int k = 0; k < trav[i].plen; k++) {
                if (trav[i].path[k] == msg.current) { trav[i].pathIdx = k; break; }
            }

            if (msg.next < 0) {
                printf("[PID=%d] arrived at node %d | next node: DESTINATION\n",
                       (int)trav[i].pid, msg.current);
                printf("[PID=%d] finished\n", (int)trav[i].pid);
                fflush(stdout);
                trav[i].state = ANIM_FINISHED;
                close(trav[i].pipe_fd[0]);
                trav[i].pipe_fd[0] = -1;
                DBG("PARENT: traveler %d finished at node %d\n", i, msg.current);
                break;
            } else {
                printf("[PID=%d] arrived at node %d | next node: %d\n",
                       (int)trav[i].pid, msg.current, msg.next);
                fflush(stdout);
                DBG("PARENT: traveler %d at node %d -> %d\n", i, msg.current, msg.next);
            }
        }

        /* EOF: child closed its write end (exited) */
        if (r == 0 && trav[i].pipe_fd[0] >= 0) {
            trav[i].state = ANIM_FINISHED;
            close(trav[i].pipe_fd[0]);
            trav[i].pipe_fd[0] = -1;
        }
        /* r == -1 with EAGAIN/EWOULDBLOCK: no more data right now -- expected */
    }
    return activity;
}

/* Kill any still-running children and reap all. */
static void cleanup_children(Traveler *trav, int n) {
    /* close remaining read ends first so children writing get SIGPIPE */
    for (int i = 0; i < n; i++) {
        if (trav[i].pipe_fd[0] >= 0) {
            close(trav[i].pipe_fd[0]);
            trav[i].pipe_fd[0] = -1;
        }
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
    DBG("PARENT: all %d children reaped, exiting.\n", n);
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
    if (T->pathIdx > 0)            return "Moving";
    return "Start";
}

static void draw_frame(const Graph *g, Traveler *trav, int n, int paused) {
    Color CN = { 55, 70, 110, 255 }, CE = { 60, 75, 115, 255 };
    Color CT = { 210, 220, 240, 255 }, CW = { 130, 155, 200, 255 };

    BeginDrawing();
    ClearBackground((Color){ 12, 15, 26, 255 });

    for (int x = 0; x < WIN_W; x += 38)
        for (int y = 0; y < WIN_H - 80; y += 38)
            DrawCircle(x, y, 1, (Color){ 30, 38, 65, 255 });

    DrawText("OS Project - Milestone 5 (pipe IPC)", 18, 12, 20,
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

    /* each traveler's route (received via MSG_PATH), tinted in its color */
    for (int i = 0; i < n; i++) {
        if (trav[i].plen < 2) continue;
        Color c = trav[i].color; c.a = 130;
        for (int e = 0; e < trav[i].plen - 1; e++)
            draw_arrow(trav[i].path[e], trav[i].path[e + 1], c, 3.0f);
    }

    /* nodes */
    for (int i = 0; i < g->numNodes; i++) {
        DrawCircle((int)nx[i], (int)ny[i], NODE_R, CN);
        DrawCircleLines((int)nx[i], (int)ny[i], NODE_R, WHITE);
        char lb[8]; snprintf(lb, sizeof lb, "%d", i);
        DrawText(lb, (int)nx[i] - MeasureText(lb, 19) / 2, (int)ny[i] - 9, 19, CT);
    }

    /* traveler dots */
    for (int i = 0; i < n; i++) {
        DrawCircle((int)trav[i].x, (int)trav[i].y, 13, trav[i].color);
        DrawCircleLines((int)trav[i].x, (int)trav[i].y, 13, BLACK);
        char id[8]; snprintf(id, sizeof id, "%d", i);
        DrawText(id, (int)trav[i].x - MeasureText(id, 14) / 2,
                     (int)trav[i].y - 7, 14, BLACK);
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
        DrawRectangle(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46, (Color){ 10, 50, 25, 230 });
        DrawRectangleLines(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46,
                           (Color){ 80, 220, 100, 255 });
        DrawText(msg, WIN_W / 2 - mw / 2, 33, 28, (Color){ 255, 240, 70, 255 });
    }

    /* bottom panel */
    DrawRectangle(0, WIN_H - 80, WIN_W, 80, (Color){ 20, 24, 40, 255 });
    DrawLine(0, WIN_H - 80, WIN_W, WIN_H - 80, (Color){ 45, 55, 90, 255 });
    int done = 0;
    for (int i = 0; i < n; i++) if (trav[i].state == ANIM_FINISHED) done++;
    char st[96]; snprintf(st, sizeof st, "Travelers: %d   Arrived: %d / %d   %s",
                          n, done, n, paused ? "[PAUSED]" : "[RUNNING]");
    DrawText(st, 18, WIN_H - 64, 18, CT);
    DrawText("[SPACE] pause/resume    close window to quit", 18, WIN_H - 38, 15, CW);
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

    /* assign colors and set initial screen positions */
    for (int i = 0; i < n; i++) {
        trav[i].color    = make_color(i, n);
        trav[i].state    = ANIM_AT_NODE;
        trav[i].pathIdx  = 0;
        trav[i].x        = nx[trav[i].src];
        trav[i].y        = ny[trav[i].src];
    }

    /* ---- create one pipe per traveler BEFORE forking ---- */
    for (int i = 0; i < n; i++) {
        if (pipe(trav[i].pipe_fd) != 0) {
            perror("pipe");
            for (int j = 0; j < i; j++) {
                close(trav[j].pipe_fd[0]); close(trav[j].pipe_fd[1]);
            }
            gfree(&g); return 1;
        }
    }

    fflush(NULL);   /* flush before fork so buffers are not duplicated */

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
            gfree(&g); return 1;
        }

        if (pid == 0) {
            /* ====== CHILD i ====== */
            /* Close all read ends and the write ends of all OTHER travelers. */
            for (int j = 0; j < n; j++) {
                close(trav[j].pipe_fd[0]);
                if (j != i) close(trav[j].pipe_fd[1]);
            }
            int wfd = trav[i].pipe_fd[1];

            /* Compute OWN Dijkstra path (graph inherited from parent via fork). */
            int path[MAX_N]; int plen = 0; long long wt = 0;
            int src = trav[i].src, dst = trav[i].dst;

            if (src == dst) {
                path[0] = src; plen = 1; wt = 0;
            } else {
                if (!dijkstra(&g, src, dst, path, &plen, &wt)){
                    TravelMsg sm;
                    memset(&sm, 0, sizeof sm);
                    sm.type = MSG_STUCK;
                    { ssize_t wr = write(wfd, &sm, sizeof sm); (void)wr; }
                    close(wfd);
                    _exit(0);
                }
    
            }

            /* Send MSG_PATH so parent can draw the route visualization. */
            TravelMsg pm;
            memset(&pm, 0, sizeof pm);
            pm.type   = MSG_PATH;
            pm.plen   = plen;
            pm.weight = wt;
            memcpy(pm.path, path, (size_t)plen * sizeof(int));
            { ssize_t w = write(wfd, &pm, sizeof pm); (void)w; }

            /* Walk the path: sleep per edge, report each node arrival. */
            for (int step = 1; step < plen; step++) {
                int prev_node = path[step - 1];
                int curr_node = path[step];
                int w = edge_weight(&g, prev_node, curr_node);
                if (w < 1) w = 1;

                /* travel time for this edge */
                usleep((useconds_t)(w * JUMP_TIME * 1000000.0));

                TravelMsg am;
                memset(&am, 0, sizeof am);
                am.type    = MSG_ARRIVE;
                am.current = curr_node;
                am.next    = (step < plen - 1) ? path[step + 1] : -1;
                { ssize_t wr = write(wfd, &am, sizeof am); (void)wr; }

                /* wait at intermediate node before next leg */
                if (step < plen - 1)
                    usleep((useconds_t)(NODE_WAIT * 1000000.0));
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
        close(trav[i].pipe_fd[1]);
        trav[i].pipe_fd[1] = -1;
    }

    /*
     * Blocking read of one MSG_PATH per child.
     * Children send MSG_PATH immediately (Dijkstra on N<=15 is instant), so
     * these reads return in microseconds and prime the path visualization
     * before the GUI loop starts.
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
                close(trav[i].pipe_fd[0]);
                trav[i].pipe_fd[0] = -1;
            }
            DBG("PARENT: traveler %d MSG_PATH plen=%d weight=%lld\n",
                i, pm.plen, pm.weight);
        } else {
            /* pipe closed before PATH (child exited early) */
            trav[i].plen  = 0;
            trav[i].state = ANIM_FINISHED;
            if (trav[i].pipe_fd[0] >= 0) { close(trav[i].pipe_fd[0]); trav[i].pipe_fd[0] = -1; }
        }
    }

    /* Switch remaining read ends to non-blocking for the main loop. */
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
    gfree(&g);
    return 0;

#else
    /* ---- GUI: parent runs the raylib loop, drains pipes each frame ---- */
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(WIN_W, WIN_H, "OS Project - Milestone 5");
    SetTargetFPS(60);

    int paused = 0;
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (!paused) drain_pipes(trav, n);
        draw_frame(&g, trav, n, paused);
    }

    cleanup_children(trav, n);
    CloseWindow();
    gfree(&g);
    return 0;
#endif
}
