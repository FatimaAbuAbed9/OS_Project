/* ============================================================================
 *  OS Project  --  Milestone 4 : multiple processes and a parent process
 * ----------------------------------------------------------------------------
 *  Model:
 *    - The PARENT reads the (extended) input file, computes a Dijkstra path
 *      for every traveler, then fork()s one CHILD per traveler.
 *    - Each CHILD prints "[<pid>] started" once and then sleeps (does nothing).
 *    - The PARENT runs the raylib loop and animates ALL travelers at the same
 *      time, each drawn in a distinct color.
 *    - When a traveler reaches its destination the PARENT sends SIGTERM to that
 *      traveler's child and reaps it (waitpid).  The PARENT waits for every
 *      child before it exits.
 *
 *  Build (see Makefile):
 *    GUI (graded):   make milestone4            -> ./sim   (needs raylib)
 *    Headless (test): make milestone4-headless  -> ./sim_test (no raylib)
 *
 *  The two builds share ALL of the logic below; only the main loop / drawing
 *  differ (guarded by #ifdef HEADLESS).  The headless build runs the exact
 *  same fork / signal / wait orchestration on a virtual clock so the behavior
 *  can be tested without a display.
 *
 *  Diagnostics: set the environment variable SIM_DEBUG=1 to print a trace of
 *  the parent's actions (fork / paths / signals / reaps) to stderr.  It is OFF
 *  by default, so the normal graded run prints nothing but the children's
 *  "started" lines, exactly as the milestone requires.
 * ==========================================================================*/

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>
#include <unistd.h>     /* fork, getpid, pause            */
#include <signal.h>     /* kill, SIGTERM, sigprocmask     */
#include <sys/wait.h>   /* waitpid                        */
#include <sys/types.h>  /* pid_t                          */

#ifndef HEADLESS
#include <raylib.h>
#else
/* Minimal stand-in so the shared structs compile without raylib. */
typedef struct { unsigned char r, g, b, a; } Color;
#endif

/* ---------------------------------------------------------------- limits -- */
#define WIN_W         1000
#define WIN_H          720
#define NODE_R          26
#define MAX_N           15      /* max graph nodes (circular layout limit)   */
#define MAX_TRAVELERS   64      /* max simultaneous travelers                */
#define PIV   3.14159265f

/* ------------------------------------------------ animation timing (m3) -- */
#define NODE_WAIT  1.0f   /* wait 1 s at each *intermediate* node            */
#define JUMP_TIME  0.3f   /* 300 ms per jump; an edge of weight W = W jumps  */

/* =================================  model  ================================ */

typedef struct { int dst; int weight; } Edge;
typedef struct { Edge *edges; int count; int capacity; } EdgeList;
typedef struct { int numNodes; int numEdges; EdgeList *adjList; } Graph;

typedef enum { ANIM_AT_NODE, ANIM_ON_EDGE, ANIM_FINISHED } AnimState;

typedef struct {
    int        src, dst;            /* requested source / destination        */
    int        path[MAX_N];         /* computed shortest path (node indices) */
    int        plen;                /* path length; 0 = unreachable          */
    long long  weight;              /* total path weight                     */

    pid_t      pid;                 /* the child process for this traveler   */
    int        signaled;            /* parent already killed + reaped it     */

    AnimState  state;               /* current animation sub-state           */
    int        pathIdx;             /* index into path[] we are at / leaving */
    int        jump;                /* completed jumps on the current edge   */
    float      timer;              /* seconds in the current sub-state      */
    int        movedOnce;           /* has it left the source yet?           */

    Color      color;               /* GUI color (distinct per traveler)     */
    float      x, y;                /* on-screen position                    */
} Traveler;

/* node screen positions (filled by layout()) */
static float nx[MAX_N], ny[MAX_N];

/* parent-only debug trace, toggled by env SIM_DEBUG */
static int g_debug = 0;
#define DBG(...) do { if (g_debug) { fprintf(stderr, __VA_ARGS__); fflush(stderr); } } while (0)

/* virtual clock used for the trace (seconds) */
static float g_simTime = 0.0f;

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
        el->edges = tmp;
        el->capacity = nc;
    }
    el->edges[el->count].dst = d;
    el->edges[el->count].weight = w;
    el->count++;
    return 1;
}

static int edge_weight(const Graph *g, int u, int v) {
    for (int k = 0; k < g->adjList[u].count; k++)
        if (g->adjList[u].edges[k].dst == v) return g->adjList[u].edges[k].weight;
    return 1; /* should not happen on a valid Dijkstra path */
}

/* O(N^2) Dijkstra -- fine because N <= MAX_N (15). Same as milestone 2/3. */
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
            int v = g->adjList[u].edges[k].dst;
            int w = g->adjList[u].edges[k].weight;
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
/*  Extended format (comment lines beginning with '#' and blank lines are
 *  ignored, so files with or without the "# graph definition" / "# travelers"
 *  headers both parse):
 *
 *      N M                 <- nodes, edges
 *      src dst weight      <- M edge lines
 *      ...
 *      T                   <- number of travelers
 *      src dst             <- T traveler lines
 */

/* Read the next data line: skips blank lines and '#' comments (incl. inline). */
static int next_line(FILE *f, char *buf, size_t cap) {
    while (fgets(buf, (int)cap, f)) {
        char *h = strchr(buf, '#');
        if (h) *h = '\0';                       /* strip inline comment */
        char *p = buf;
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p != '\0') return 1;               /* found real content   */
    }
    return 0;
}

static int load_input(Graph *g, const char *fn, Traveler *trav, int *numTrav) {
    FILE *f = fopen(fn, "r");
    if (!f) { fprintf(stderr, "Error: cannot open file '%s'.\n", fn); return 0; }

    char line[256];
    int  N, M;

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

/* =====================  per-traveler animation (m3 logic)  ================ */

static void traveler_init(Traveler *T) {
    T->state = ANIM_AT_NODE; T->pathIdx = 0; T->jump = 0; T->timer = 0.0f;
    T->signaled = 0; T->movedOnce = 0;
    if (T->plen >= 1) { T->x = nx[T->path[0]]; T->y = ny[T->path[0]]; }
}

/* Advance one traveler by dt seconds. Returns 1 if it just left its source. */
static int traveler_update(const Graph *g, Traveler *T, float dt) {
    int firstMove = 0;
    if (T->plen <= 1) { T->state = ANIM_FINISHED; return 0; } /* src==dst / unreachable */
    if (T->state == ANIM_FINISHED) return 0;

    T->timer += dt;

    if (T->state == ANIM_AT_NODE) {
        if (T->pathIdx >= T->plen - 1) { T->state = ANIM_FINISHED; return 0; }
        if (T->pathIdx == 0) {                       /* source: leave at once */
            T->state = ANIM_ON_EDGE; T->jump = 0; T->timer = 0.0f;
            if (!T->movedOnce) { T->movedOnce = 1; firstMove = 1; }
        } else if (T->timer >= NODE_WAIT) {          /* intermediate: wait 1s */
            T->state = ANIM_ON_EDGE; T->jump = 0; T->timer = 0.0f;
        }
    }

    if (T->state == ANIM_ON_EDGE) {
        int u = T->path[T->pathIdx], v = T->path[T->pathIdx + 1];
        int W = edge_weight(g, u, v); if (W < 1) W = 1;
        while (T->timer >= JUMP_TIME && T->state == ANIM_ON_EDGE) {
            T->timer -= JUMP_TIME; T->jump++;
            if (T->jump >= W) {                       /* reached next node */
                T->pathIdx++; T->state = ANIM_AT_NODE; T->jump = 0; T->timer = 0.0f;
            }
        }
    }
    return firstMove;
}

static void traveler_position(const Graph *g, Traveler *T) {
    if (T->plen == 0) return;
    if (T->state == ANIM_AT_NODE || T->state == ANIM_FINISHED) {
        int n = T->path[T->pathIdx]; T->x = nx[n]; T->y = ny[n];
    } else {
        int u = T->path[T->pathIdx], v = T->path[T->pathIdx + 1];
        int W = edge_weight(g, u, v); if (W < 1) W = 1;
        float sub = T->timer / JUMP_TIME; if (sub > 1.0f) sub = 1.0f;
        float t = ((float)T->jump + sub) / (float)W; if (t > 1.0f) t = 1.0f;
        T->x = nx[u] + (nx[v] - nx[u]) * t;
        T->y = ny[u] + (ny[v] - ny[u]) * t;
    }
}

/* ========================  process orchestration  ======================== */

static int all_finished(const Traveler *t, int n) {
    for (int i = 0; i < n; i++) if (t[i].state != ANIM_FINISHED) return 0;
    return 1;
}

/* Kill + reap a single child (idempotent via the `signaled` flag). */
static void reap_one(Traveler *T, int idx, const char *why) {
    if (T->signaled) return;
    DBG("PARENT: traveler %d finished; KILL pid=%d (SIGTERM) [%s]\n", idx, (int)T->pid, why);
    kill(T->pid, SIGTERM);
    int st; waitpid(T->pid, &st, 0);
    T->signaled = 1;
    DBG("PARENT: REAP pid=%d (traveler %d)\n", (int)T->pid, idx);
}

/* One simulation frame for every traveler. On a finish: signal + reap. */
static void sim_step(const Graph *g, Traveler *trav, int n, float dt) {
    for (int i = 0; i < n; i++) {
        Traveler *T = &trav[i];
        int before = T->pathIdx;
        AnimState bs = T->state;

        int firstMove = traveler_update(g, T, dt);
        traveler_position(g, T);

        if (firstMove)
            DBG("SIM t=%.2f traveler%d FIRST_MOVE from node %d\n", g_simTime, i, T->path[0]);

        if (T->pathIdx != before) {                  /* arrived at a new node */
            int node = T->path[T->pathIdx];
            if (T->pathIdx >= T->plen - 1)
                DBG("SIM t=%.2f traveler%d arrived node %d DEST\n", g_simTime, i, node);
            else
                DBG("SIM t=%.2f traveler%d arrived node %d (next %d)\n",
                    g_simTime, i, node, T->path[T->pathIdx + 1]);
        }

        if (T->state == ANIM_FINISHED && bs != ANIM_FINISHED)
            reap_one(T, i, "arrived");
        else if (T->state == ANIM_FINISHED && !T->signaled)
            reap_one(T, i, "already-finished");
    }
    g_simTime += dt;
}

/* Final cleanup: make sure every child is dead and reaped before we exit. */
static void cleanup_children(Traveler *trav, int n) {
    for (int i = 0; i < n; i++)
        if (!trav[i].signaled) reap_one(&trav[i], i, "cleanup");
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

static int on_path(const Traveler *T, int u, int v) {
    for (int i = 0; i < T->plen - 1; i++) if (T->path[i] == u && T->path[i + 1] == v) return 1;
    return 0;
}

static const char *state_word(const Traveler *T) {
    if (T->state == ANIM_FINISHED) return "Arrived";
    if (T->state == ANIM_ON_EDGE)  return "Moving";
    return (T->pathIdx == 0) ? "Start" : "Waiting";
}

static void draw_frame(const Graph *g, Traveler *trav, int n, int paused) {
    Color CN = { 55, 70, 110, 255 }, CE = { 60, 75, 115, 255 };
    Color CT = { 210, 220, 240, 255 }, CW = { 130, 155, 200, 255 };

    BeginDrawing();
    ClearBackground((Color){ 12, 15, 26, 255 });

    for (int x = 0; x < WIN_W; x += 38)
        for (int y = 0; y < WIN_H - 80; y += 38)
            DrawCircle(x, y, 1, (Color){ 30, 38, 65, 255 });

    DrawText("OS Project - Milestone 4 (multiple travelers)", 18, 12, 20,
             (Color){ 90, 115, 185, 255 });

    /* neutral edges + weights */
    for (int u = 0; u < g->numNodes; u++)
        for (int k = 0; k < g->adjList[u].count; k++) {
            int v = g->adjList[u].edges[k].dst, w = g->adjList[u].edges[k].weight;
            draw_arrow(u, v, CE, 1.6f);
            char ws[8]; snprintf(ws, sizeof ws, "%d", w);
            DrawText(ws, (int)((nx[u] + nx[v]) / 2 + 10), (int)((ny[u] + ny[v]) / 2 - 10), 15, CW);
        }

    /* each traveler's route, tinted faintly in its own color */
    for (int i = 0; i < n; i++) {
        Color c = trav[i].color; c.a = 130;
        for (int e = 0; e < trav[i].plen - 1; e++)
            draw_arrow(trav[i].path[e], trav[i].path[e + 1], c, 3.0f);
        (void)on_path;
    }

    /* nodes */
    for (int i = 0; i < g->numNodes; i++) {
        DrawCircle((int)nx[i], (int)ny[i], NODE_R, CN);
        DrawCircleLines((int)nx[i], (int)ny[i], NODE_R, WHITE);
        char lb[8]; snprintf(lb, sizeof lb, "%d", i);
        DrawText(lb, (int)nx[i] - MeasureText(lb, 19) / 2, (int)ny[i] - 9, 19, CT);
    }

    /* travelers (moving dots), drawn on top, each in its color with its index */
    for (int i = 0; i < n; i++) {
        if (trav[i].plen < 1) continue;
        DrawCircle((int)trav[i].x, (int)trav[i].y, 13, trav[i].color);
        DrawCircleLines((int)trav[i].x, (int)trav[i].y, 13, BLACK);
        char id[8]; snprintf(id, sizeof id, "%d", i);
        DrawText(id, (int)trav[i].x - MeasureText(id, 14) / 2, (int)trav[i].y - 7, 14, BLACK);
    }

    /* legend (top-left, one compact row per traveler) */
    int lx = 18, ly = 40;
    for (int i = 0; i < n && i < 12; i++) {
        DrawRectangle(lx, ly + i * 20 + 2, 14, 14, trav[i].color);
        DrawRectangleLines(lx, ly + i * 20 + 2, 14, 14, WHITE);
        char row[96];
        if (trav[i].plen == 0)
            snprintf(row, sizeof row, "T%d  %d->%d  (no path)", i, trav[i].src, trav[i].dst);
        else
            snprintf(row, sizeof row, "T%d  %d->%d  %s", i, trav[i].src, trav[i].dst, state_word(&trav[i]));
        DrawText(row, lx + 20, ly + i * 20, 15, CT);
    }

    /* all-arrived banner */
    if (all_finished(trav, n)) {
        const char *msg = "All travelers arrived!";
        int mw = MeasureText(msg, 28);
        DrawRectangle(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46, (Color){ 10, 50, 25, 230 });
        DrawRectangleLines(WIN_W / 2 - mw / 2 - 22, 24, mw + 44, 46, (Color){ 80, 220, 100, 255 });
        DrawText(msg, WIN_W / 2 - mw / 2, 33, 28, (Color){ 255, 240, 70, 255 });
    }

    /* bottom panel: status + hint */
    DrawRectangle(0, WIN_H - 80, WIN_W, 80, (Color){ 20, 24, 40, 255 });
    DrawLine(0, WIN_H - 80, WIN_W, WIN_H - 80, (Color){ 45, 55, 90, 255 });

    int done = 0; for (int i = 0; i < n; i++) if (trav[i].state == ANIM_FINISHED) done++;
    char st[96]; snprintf(st, sizeof st, "Travelers: %d   Arrived: %d / %d   %s",
                          n, done, n, paused ? "[PAUSED]" : "[RUNNING]");
    DrawText(st, 18, WIN_H - 64, 18, CT);
    DrawText("[SPACE] pause/resume    close window to quit", 18, WIN_H - 38, 15, CW);

    EndDrawing();
}
#endif /* !HEADLESS */

/* ================================  main  ================================= */

int main(int argc, char *argv[]) {
    if (argc < 2) { fprintf(stderr, "Usage: %s <file_name>\n", argv[0]); return 1; }
    g_debug = (getenv("SIM_DEBUG") != NULL);

    Graph g; ginit(&g);
    Traveler trav[MAX_TRAVELERS];
    int n = 0;
    if (!load_input(&g, argv[1], trav, &n)) { gfree(&g); return 1; }

    layout(g.numNodes);

    /* ---- PARENT computes every traveler's path BEFORE forking ---- */
    for (int i = 0; i < n; i++) {
        if (trav[i].src == trav[i].dst) {
            trav[i].path[0] = trav[i].src; trav[i].plen = 1; trav[i].weight = 0;
        } else if (!dijkstra(&g, trav[i].src, trav[i].dst,
                             trav[i].path, &trav[i].plen, &trav[i].weight)) {
            trav[i].plen = 0; trav[i].weight = 0;        /* unreachable */
        }
        trav[i].color = make_color(i, n);
        traveler_init(&trav[i]);

        if (g_debug) {
            char ps[512] = "";
            if (trav[i].plen == 0) snprintf(ps, sizeof ps, "NO_PATH");
            else {
                size_t off = 0;
                for (int k = 0; k < trav[i].plen; k++)
                    off += (size_t)snprintf(ps + off, sizeof ps - off, "%s%d",
                                            k ? " -> " : "", trav[i].path[k]);
            }
            DBG("PARENT: traveler %d path: %s (weight %lld)\n", i, ps, trav[i].weight);
        }
    }

    /* ---- fork one child per traveler ----
     * A small pipe is used purely as a STARTUP BARRIER: each child writes one
     * byte right after printing "[pid] started", and the parent waits until it
     * has read one byte per child before it begins the simulation -- and thus
     * before it can signal anyone. Without this, the parent (which in headless
     * mode runs on a fast virtual clock) could SIGTERM a child before that child
     * was ever scheduled to run, and the default action would terminate it
     * before it printed "started". The barrier carries no path data; it only
     * synchronizes startup. */
    int bar[2];
    if (pipe(bar) != 0) { perror("pipe"); gfree(&g); return 1; }

    fflush(NULL);                                        /* avoid inherited buffers */
    int forked = 0;
    for (int i = 0; i < n; i++) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("fork");
            close(bar[0]); close(bar[1]);
            for (int j = 0; j < forked; j++) reap_one(&trav[j], j, "fork-error");
            gfree(&g);
            return 1;
        }
        if (pid == 0) {
            /* ====== CHILD ====== */
            close(bar[0]);                               /* child only writes */
            printf("[%d] started\n", (int)getpid());
            fflush(stdout);
            char b = 1; ssize_t wr = write(bar[1], &b, 1); (void)wr;  /* "I started" */
            close(bar[1]);
            for (;;) pause();                            /* sleep until parent kills us */
            _exit(0);                                    /* not reached */
        }
        /* ====== PARENT ====== */
        trav[i].pid = pid;
        forked++;
        DBG("PARENT: forked child pid=%d for traveler %d (%d -> %d)\n",
            (int)pid, i, trav[i].src, trav[i].dst);
    }

    /* barrier: block until every child has printed "started" */
    close(bar[1]);                                       /* parent does not write */
    {
        char b; int got = 0;
        while (got < forked && read(bar[0], &b, 1) == 1) got++;
    }
    close(bar[0]);
    DBG("PARENT: all %d children reported 'started'\n", forked);

#ifdef HEADLESS
    /* ---- headless: same orchestration on a fixed virtual clock ---- */
    const float dt = 0.05f;                              /* divides 0.3 and 1.0 cleanly */
    long guard = 0;
    while (!all_finished(trav, n)) {
        sim_step(&g, trav, n, dt);
        if (++guard > 1000000) { DBG("PARENT: guard limit hit (bug?)\n"); break; }
    }
    cleanup_children(trav, n);
    gfree(&g);
    return 0;
#else
    /* ---- GUI: the parent runs the raylib loop and animates all travelers ---- */
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(WIN_W, WIN_H, "OS Project - Milestone 4");
    SetTargetFPS(60);

    int paused = 0;
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        float dt = paused ? 0.0f : GetFrameTime();
        sim_step(&g, trav, n, dt);
        draw_frame(&g, trav, n, paused);
    }

    cleanup_children(trav, n);                           /* kill+reap any survivors */
    CloseWindow();
    gfree(&g);
    return 0;
#endif
}
